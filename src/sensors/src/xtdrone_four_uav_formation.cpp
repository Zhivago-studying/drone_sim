/**
 * @file  xtdrone_four_uav_formation.cpp
 * @brief XTDrone 四机编队飞行控制器 (Section 5.2)
 *
 * =========================== 功能描述 ===========================
 * 实现面向 XTDrone 仿真平台的 4 机编队飞行控制器。包含 7 阶段状态机：
 * WAIT → TAKEOFF → HOVER → EXPAND_SHRINK → TRANSLATE → CHASE_RESTORE → LAND → DONE。
 * 支持起飞稳定等待、编队扩张-收缩、整体平移、位置追逐与复原等多模态编队机动，
 * 以及自动重试 OFFBOARD/ARM 的容错机制。
 *
 * =========================== 数据流 ===========================
 * 订阅:
 *   /gazebo/model_states           (gazebo_msgs/ModelStates)
 *     - 获取所有无人机 Gazebo 真值位置，用于闭环控制
 *   /iris_X/mavros/state           (mavros_msgs/State)
 *     - 监控各无人机连接、解锁、模式状态
 *   /iris_X/mavros/local_position/pose (geometry_msgs/PoseStamped)
 *     - 获取每架无人机 MAVROS local 坐标原点下的位置和初始 yaw
 *
 * 发布:
 *   /iris_X/mavros/setpoint_position/local  (geometry_msgs/PoseStamped)
 *     - 起飞/悬停/降落阶段的位置控制指令
 *   /iris_X/mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped)
 *     - 编队机动阶段的速度控制指令 (比例导引)
 *
 * 服务调用:
 *   /iris_X/mavros/set_mode    (mavros_msgs/SetMode)      — OFFBOARD
 *   /iris_X/mavros/cmd/arming  (mavros_msgs/CommandBool)  — 解锁/上锁
 *
 * =========================== 7 阶段状态机 ===========================
 *   WAIT_FOR_GAZEBO  — 等待 Gazebo 真值数据 + MAVROS 连接就绪
 *   TAKEOFF          — 爬升至 kFlightZ=3.0m，稳定 1s 后推进
 *   HOVER            — 悬停 2s
 *   EXPAND_SHRINK    — 沿径向扩张 2.0m 再收缩回原位 (5s + 5s)
 *   TRANSLATE        — 整体沿 ENU +Y 方向平移 3.0m 再返回 (5s + 5s)
 *   CHASE_RESTORE    — iris_0→p1, iris_1→p2, iris_2→p3, iris_3→p0，再复原 (5s + 5s)
 *   LAND / DONE      — 逐步下降至 0m，上锁退出
 *
 * =========================== 关键参数 ===========================
 *   - 控制频率: 20 Hz
 *   - 最大速度: 1.0 m/s
 *   - 起飞超时: 12.0 s (可通过参数设置)
 *   - 降落超时: 20.0 s (可通过参数设置)
 *   - 可配置参数: ~takeoff_timeout, ~landing_timeout
 *
 * =========================== 启动方式 ===========================
 *   单节点集中式控制:
 *   <node pkg="sensors" type="xtdrone_four_uav_formation" name="xtdrone_controller">
 *     <param name="takeoff_timeout" value="12.0"/>
 *     <param name="landing_timeout" value="20.0"/>
 *   </node>
 */

#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>

#include <boost/filesystem.hpp>

struct Vec3
{
  double x;
  double y;
  double z;
};

Vec3 opAdd(const Vec3& a, const Vec3& b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 opSub(const Vec3& a, const Vec3& b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 opMul(const Vec3& a, double s)
{
  return {a.x * s, a.y * s, a.z * s};
}

double length(const Vec3& v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

bool zero(const Vec3& v)
{
  return length(v) < 1.0e-6;
}

Vec3 normalize(const Vec3& v)
{
  if (zero(v))
  {
    return {0.0, 0.0, 0.0};
  }
  return opMul(v, 1.0 / length(v));
}

namespace
{
constexpr int kUavCount = 4;
constexpr double kRateHz = 20.0;
constexpr double kFlightZ = 3.0;
constexpr double kMaxSpeed = 1.0;
constexpr double kTakeoffAltitudeTolerance = 0.15;
constexpr double kStageAltitudeTolerance = 0.25;

const std::array<std::string, kUavCount> kModelNames = {
  "iris_0", "iris_1", "iris_2", "iris_3"
};

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

double norm2d(const Vec3& v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

Vec3 normalize2d(const Vec3& v)
{
  const double n = norm2d(v);
  if (n < 1.0e-6)
  {
    return {0.0, 0.0, 0.0};
  }
  return {v.x / n, v.y / n, 0.0};
}

double yawFromPose(const geometry_msgs::PoseStamped& pose)
{
  const auto& q = pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class XtdroneFormationController
{
public:
  XtdroneFormationController()
    : nh_(),
      private_nh_("~"),
      model_received_(false),
      cached_pose_active_(false),
      cached_twist_active_(false),
      stage_(Stage::WAIT_FOR_GAZEBO),
      motion_subphase_(MotionSubphase::OUTBOUND),
      landing_target_z_(kFlightZ),
      landing_start_z_(kFlightZ),
      landing_descent_rate_(0.45),
      force_disarm_delay_(2.0),
      takeoff_target_z_(kFlightZ),
      takeoff_altitude_ready_since_(0.0),
      last_ready_retry_(0.0),
      last_disarm_request_(0.0)
  {
    local_pose_received_.fill(false);
    local_yaws_.fill(0.0);
    home_yaws_.fill(0.0);
    for (int i = 0; i < kUavCount; ++i)
    {
      armed_flags_[i].store(false);
    }

    gazebo_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                                &XtdroneFormationController::gazeboCb, this);
    stage_pub_ = nh_.advertise<std_msgs::UInt8>("/formation/stage", 1, true);
    publishStage();

    for (int i = 0; i < kUavCount; ++i)
    {
      const std::string mavros_ns = "/" + kModelNames[i] + "/mavros";
      pose_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
        mavros_ns + "/setpoint_position/local", 10);
      twist_pubs_[i] = nh_.advertise<geometry_msgs::TwistStamped>(
        mavros_ns + "/setpoint_velocity/cmd_vel", 10);
      state_subs_[i] = nh_.subscribe<mavros_msgs::State>(
        mavros_ns + "/state", 10,
        boost::bind(&XtdroneFormationController::stateCb, this, _1, i));
      local_pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(
        mavros_ns + "/local_position/pose", 10,
        boost::bind(&XtdroneFormationController::localPoseCb, this, _1, i));
      arm_clients_[i] = nh_.serviceClient<mavros_msgs::CommandBool>(
        mavros_ns + "/cmd/arming");
      command_clients_[i] = nh_.serviceClient<mavros_msgs::CommandLong>(
        mavros_ns + "/cmd/command");
      mode_clients_[i] = nh_.serviceClient<mavros_msgs::SetMode>(
        mavros_ns + "/set_mode");
      param_clients_[i] = nh_.serviceClient<mavros_msgs::ParamSet>(
        mavros_ns + "/param/set");
    }

    private_nh_.param("takeoff_timeout", takeoff_timeout_, 12.0);
    private_nh_.param("landing_timeout", landing_timeout_, 20.0);
    private_nh_.param("takeoff_climb_rate", takeoff_climb_rate_, 0.45);
    private_nh_.param("landing_descent_rate", landing_descent_rate_, 0.45);
    private_nh_.param("force_disarm_delay", force_disarm_delay_, 2.0);
    // 论文 1 m/s 动态运动参数
    private_nh_.param("horizontal_speed", horizontal_speed_, 1.0);
    private_nh_.param("max_horizontal_speed", max_horizontal_speed_, 1.0);
    private_nh_.param("slow_radius", slow_radius_, 0.80);
    private_nh_.param("stage_target_tolerance", stage_target_tolerance_, 0.25);
    private_nh_.param("min_pairwise_distance", min_pairwise_distance_, 0.8);
    // 阶段缓冲参数
    private_nh_.param("phase_settle_time", phase_settle_time_, 1.0);
    private_nh_.param("stage_settle_time", stage_settle_time_, 1.0);
    private_nh_.param("turnaround_settle_time", turnaround_settle_time_, 0.2);
    // 加速度限制
    private_nh_.param("max_accel_xy", max_accel_xy_, 0.6);
    // 各阶段独立速度
    private_nh_.param("translate_speed", translate_speed_, 0.8);
    private_nh_.param("chase_speed", chase_speed_, 0.6);
    private_nh_.param("chase_slow_radius", chase_slow_radius_, 1.0);
    private_nh_.param("arrive_speed_threshold", arrive_speed_threshold_, 0.20);
    // 低速保持控制
    private_nh_.param("settle_p_gain", settle_p_gain_, 0.4);
    private_nh_.param("settle_speed_max", settle_speed_max_, 0.25);
    private_nh_.param("settle_pos_tolerance", settle_pos_tolerance_, 0.25);
    private_nh_.param("settle_speed_tolerance", settle_speed_tolerance_, 0.20);
    // Translate 整体平移控制
    private_nh_.param("translate_shape_gain", translate_shape_gain_, 0.15);
    // EXPAND_SHRINK 独立参数
    private_nh_.param("expand_speed", expand_speed_, 0.8);
    private_nh_.param("expand_return_speed", expand_return_speed_, 0.7);
    private_nh_.param("expand_slow_radius", expand_slow_radius_, 1.4);
    private_nh_.param("expand_return_slow_radius", expand_return_slow_radius_, 1.6);
    private_nh_.param("expand_brake_accel_xy", expand_brake_accel_xy_, 0.8);
    // 非对称加减速
    private_nh_.param("max_decel_xy", max_decel_xy_, 1.8);
    // 速度阻尼
    private_nh_.param("velocity_damping_gain", velocity_damping_gain_, 0.6);
    private_nh_.param("speed_csv_dir", speed_csv_dir_,
                      std::string("/home/scott/swarm_localization/src/sensors/logs"));

    if (horizontal_speed_ <= 0.0 ||
        max_horizontal_speed_ <= 0.0 ||
        slow_radius_ <= 0.0 ||
        stage_target_tolerance_ <= 0.0 ||
        min_pairwise_distance_ <= 0.0 ||
        phase_settle_time_ <= 0.0 ||
        stage_settle_time_ <= 0.0 ||
        turnaround_settle_time_ < 0.0 ||
        max_accel_xy_ <= 0.0 ||
        translate_speed_ <= 0.0 ||
        chase_speed_ <= 0.0 ||
        chase_slow_radius_ <= 0.0 ||
        arrive_speed_threshold_ <= 0.0 ||
        settle_p_gain_ <= 0.0 ||
        settle_speed_max_ <= 0.0 ||
        settle_pos_tolerance_ <= 0.0 ||
        settle_speed_tolerance_ <= 0.0 ||
        translate_shape_gain_ < 0.0 ||
        expand_speed_ <= 0.0 ||
        expand_return_speed_ <= 0.0 ||
        expand_slow_radius_ <= 0.0 ||
        expand_return_slow_radius_ <= 0.0 ||
        expand_brake_accel_xy_ <= 0.0 ||
        max_decel_xy_ <= 0.0 ||
        velocity_damping_gain_ < 0.0)
    {
      ROS_FATAL("[formation] invalid speed params: horizontal_speed=%.3f "
                "max_horizontal_speed=%.3f slow_radius=%.3f "
                "stage_target_tolerance=%.3f min_pairwise_distance=%.3f "
                "phase_settle_time=%.3f stage_settle_time=%.3f max_accel_xy=%.3f "
                "translate_speed=%.3f chase_speed=%.3f chase_slow_radius=%.3f "
                "arrive_speed_threshold=%.3f settle_p_gain=%.3f "
                "settle_speed_max=%.3f settle_pos_tolerance=%.3f "
                "settle_speed_tolerance=%.3f translate_shape_gain=%.3f "
                "turnaround_settle_time=%.3f "
                "expand_speed=%.3f expand_return_speed=%.3f "
                "expand_slow_radius=%.3f expand_return_slow_radius=%.3f "
                "expand_brake_accel_xy=%.3f max_decel_xy=%.3f "
                "velocity_damping_gain=%.3f",
                horizontal_speed_, max_horizontal_speed_,
                slow_radius_, stage_target_tolerance_,
                min_pairwise_distance_,
                phase_settle_time_, stage_settle_time_, max_accel_xy_,
                translate_speed_, chase_speed_, chase_slow_radius_,
                arrive_speed_threshold_,
                settle_p_gain_, settle_speed_max_,
                settle_pos_tolerance_, settle_speed_tolerance_,
                translate_shape_gain_,
                turnaround_settle_time_,
                expand_speed_, expand_return_speed_,
                expand_slow_radius_, expand_return_slow_radius_,
                expand_brake_accel_xy_, max_decel_xy_,
                velocity_damping_gain_);
      ros::shutdown();
      return;
    }
    horizontal_speed_ = std::min(horizontal_speed_, max_horizontal_speed_);

    initSpeedCsv();

    // Service calls and DGO/vision load can delay the mission loop. Keep the
    // last commanded setpoint on an independent callback thread so PX4 never
    // sees an OFFBOARD stream gap longer than COM_OF_LOSS_T.
    setpoint_timer_ = nh_.createTimer(
      ros::Duration(1.0 / 30.0),
      &XtdroneFormationController::setpointTimerCb, this);
  }

  void spin()
  {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(kRateHz);

    ROS_INFO("[formation] waiting for /gazebo/model_states, MAVROS states, and local poses");
    while (ros::ok() && (!model_received_ || !statesConnected() || !localPosesReceived()))
    {
      ROS_DEBUG_THROTTLE(2.0, "[formation] model_received=%d connected=%d local_pose=%d",
                         model_received_, statesConnected(), localPosesReceived());
      rate.sleep();
    }

    for (int i = 0; i < kUavCount; ++i)
    {
      home_[i] = positions_[i];
      home_[i].z = kFlightZ;
      local_home_[i] = local_positions_[i];
      local_home_[i].z = kFlightZ;
      home_yaws_[i] = local_yaws_[i];
    }
    center_ = formationCenter(home_);
    buildMotionTargets();

    ROS_INFO("[formation] home positions captured, gazebo center=(%.2f, %.2f); local position setpoints use each UAV local XY and initial yaw",
             center_.x, center_.y);

    configurePx4Params();
    primeSetpoints(rate);

    ROS_INFO("[formation] requesting OFFBOARD mode and arming");
    // 统一连续循环: 每帧并行发布 4 车 setpoint (保持 OFFBOARD 不掉线),
    // 周期性发送 set_mode/arm 并验证 /mavros/state 真正达成,直到 allReady() 或超时。
    // 取代原先串行 setMode×4 → streamFor → arm×4 → waitForAllReady 的阻塞链,
    // 后者在逐车处理时会让其它车 setpoint 断流 > COM_OF_LOSS_T,触发 OFFBOARD loss。
    bringUpOffboard(rate, 10.0);
    if (!allReady())
    {
      ROS_WARN("[formation] not all vehicles report armed OFFBOARD yet; takeoff will keep retrying");
      logVehicleStates();
    }

    enterStage(Stage::TAKEOFF);

    while (ros::ok())
    {
      switch (stage_)
      {
        case Stage::TAKEOFF:
          runTakeoff();
          break;
        case Stage::HOVER:
          runHover();
          break;
        case Stage::EXPAND_SHRINK:
          runExpandShrink();
          break;
        case Stage::TRANSLATE:
          runTranslate();
          break;
        case Stage::CHASE_RESTORE:
          runChaseRestore();
          break;
        case Stage::LAND:
          runLand();
          break;
        case Stage::DONE:
          runDisarm();
          if (allDisarmed())
          {
            ROS_INFO("[formation] mission complete, all UAVs landed and disarmed");
            return;
          }
          break;
        case Stage::WAIT_FOR_GAZEBO:
          break;
      }

      rate.sleep();
    }

    disarmAll();
  }

private:
  enum class Stage
  {
    WAIT_FOR_GAZEBO,
    TAKEOFF,
    HOVER,
    EXPAND_SHRINK,
    TRANSLATE,
    CHASE_RESTORE,
    LAND,
    DONE
  };

  enum class MotionSubphase
  {
    OUTBOUND,
    OUTBOUND_SETTLE,
    RETURN,
    RETURN_SETTLE
  };

  void gazeboCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    std::array<bool, kUavCount> seen = {false, false, false, false};

    for (std::size_t j = 0; j < msg->name.size(); ++j)
    {
      for (int i = 0; i < kUavCount; ++i)
      {
        if (msg->name[j] == kModelNames[i])
        {
          positions_[i] = {msg->pose[j].position.x,
                           msg->pose[j].position.y,
                           msg->pose[j].position.z};
          // 从 Gazebo ModelStates 读取真值速度, 用于稳定判断
          if (j < msg->twist.size())
          {
            velocities_[i].x = msg->twist[j].linear.x;
            velocities_[i].y = msg->twist[j].linear.y;
            velocities_[i].z = msg->twist[j].linear.z;
          }
          seen[i] = true;
          break;
        }
      }
    }

    model_received_ = true;
    for (int i = 0; i < kUavCount; ++i)
    {
      model_received_ = model_received_ && seen[i];
    }
  }

  void stateCb(const mavros_msgs::State::ConstPtr& msg, int i)
  {
    states_[i] = *msg;
    armed_flags_[i].store(msg->armed);
  }

  void localPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
  {
    local_positions_[i] = {msg->pose.position.x,
                           msg->pose.position.y,
                           msg->pose.position.z};
    local_yaws_[i] = yawFromPose(*msg);
    local_pose_received_[i] = true;
  }

  geometry_msgs::PoseStamped toPose(double x, double y, double z, double yaw)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    pose.pose.orientation.w = std::cos(yaw * 0.5);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    return pose;
  }

  geometry_msgs::TwistStamped toTwist(double vx, double vy, double vz)
  {
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "map";
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    return twist;
  }

  bool requestOffboardOnce(int i)
  {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = "OFFBOARD";
    return mode_clients_[i].call(srv) && srv.response.mode_sent;
  }

  bool requestArmOnce(int i)
  {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    return arm_clients_[i].call(srv) && srv.response.success;
  }

  void configurePx4Params()
  {
    ROS_INFO("[formation] configuring PX4 OFFBOARD/failsafe parameters");
    for (int i = 0; i < kUavCount; ++i)
    {
      setPx4RealParam(i, "COM_OF_LOSS_T", 5.0);
      setPx4IntParam(i, "COM_OBL_RC_ACT", 0);
      setPx4IntParam(i, "COM_RCL_EXCEPT", 4);
      setPx4IntParam(i, "COM_RC_IN_MODE", 1);
      setPx4IntParam(i, "COM_ARM_WO_GPS", 1);
      setPx4IntParam(i, "NAV_RCL_ACT", 0);
    }
  }

  bool setPx4IntParam(int i, const std::string& name, int value)
  {
    mavros_msgs::ParamSet srv;
    srv.request.param_id = name;
    srv.request.value.integer = value;
    srv.request.value.real = 0.0;
    if (param_clients_[i].call(srv) && srv.response.success)
    {
      return true;
    }
    ROS_WARN("[formation] %s failed to set PX4 param %s=%d",
             kModelNames[i].c_str(), name.c_str(), value);
    return false;
  }

  bool setPx4RealParam(int i, const std::string& name, double value)
  {
    mavros_msgs::ParamSet srv;
    srv.request.param_id = name;
    srv.request.value.integer = 0;
    srv.request.value.real = value;
    if (param_clients_[i].call(srv) && srv.response.success)
    {
      return true;
    }
    ROS_WARN("[formation] %s failed to set PX4 param %s=%.2f",
             kModelNames[i].c_str(), name.c_str(), value);
    return false;
  }

  void waitForAllReady(ros::Rate& rate, double timeout)
  {
    const ros::Time start = ros::Time::now();
    ros::Time last_retry = ros::Time(0);

    while (ros::ok() && !allReady() && (ros::Time::now() - start).toSec() < timeout)
    {
      publishCurrentPoseSetpoints();

      if ((ros::Time::now() - last_retry).toSec() > 1.0)
      {
        for (int i = 0; i < kUavCount; ++i)
        {
          if (states_[i].connected && states_[i].mode != "OFFBOARD")
          {
            requestOffboardOnce(i);
          }
          if (states_[i].connected && !states_[i].armed)
          {
            requestArmOnce(i);
          }
        }
        last_retry = ros::Time::now();
        logVehicleStates();
      }

      rate.sleep();
    }
  }

  // 起飞前统一连续循环: 每帧并行发布 4 车 setpoint, 周期性推进 set_mode/arm 并验证 state。
  // 关键: setpoint 流始终连续 (20Hz), 不会因逐车服务调用而断流, 避免 PX4 offboard loss。
  void bringUpOffboard(ros::Rate& rate, double timeout)
  {
    const ros::Time start = ros::Time::now();
    ros::Time last_retry = ros::Time(0);

    while (ros::ok() && !allReady() && (ros::Time::now() - start).toSec() < timeout)
    {
      // 每帧并行发布 4 车当前 setpoint, 保证 OFFBOARD 模式 setpoint 间隔 < COM_OF_LOSS_T
      publishCurrentPoseSetpoints();

      // 周期性 (0.5s) 发送 set_mode/arm, 非阻塞, 发完即返回
      if ((ros::Time::now() - last_retry).toSec() > 0.5)
      {
        for (int i = 0; i < kUavCount; ++i)
        {
          if (!states_[i].connected)
          {
            continue;
          }
          // Strict two-phase handshake: the set_mode service response only
          // means that PX4 accepted the request. Do not arm until a later
          // /mavros/state callback confirms that OFFBOARD is actually active.
          if (states_[i].mode != "OFFBOARD")
          {
            requestOffboardOnce(i);
          }
          else if (!states_[i].armed)
          {
            requestArmOnce(i);
          }
        }
        last_retry = ros::Time::now();
        logVehicleStates();
      }

      rate.sleep();
    }
  }

  void reassertReadyForFlight()
  {
    if (stage_ == Stage::TAKEOFF)
    {
      publishTakeoffTargets();
    }
    else
    {
      publishLocalHomePoseSetpoints();
    }

    for (int i = 0; i < kUavCount; ++i)
    {
      if (!states_[i].connected)
      {
        continue;
      }

      if (states_[i].mode != "OFFBOARD")
      {
        if (requestOffboardOnce(i))
        {
          ROS_DEBUG("[formation] %s OFFBOARD re-request accepted", kModelNames[i].c_str());
        }
        else
        {
          ROS_WARN("[formation] %s OFFBOARD re-request failed", kModelNames[i].c_str());
        }
      }

      // As above, never send ARM while PX4 still reports AUTO.RTL/POSCTL or
      // another transitional mode. PX4 rejects that request with
      // "Mode not suitable for takeoff" and may re-enter failsafe.
      if (states_[i].mode == "OFFBOARD" && !states_[i].armed)
      {
        if (requestArmOnce(i))
        {
          ROS_DEBUG("[formation] %s arm re-request accepted", kModelNames[i].c_str());
        }
        else
        {
          ROS_WARN("[formation] %s arm re-request failed", kModelNames[i].c_str());
        }
      }
    }
  }

  void logVehicleStates() const
  {
    ROS_DEBUG("[formation] states: %s[%s armed=%d conn=%d] %s[%s armed=%d conn=%d] %s[%s armed=%d conn=%d] %s[%s armed=%d conn=%d]",
             kModelNames[0].c_str(), states_[0].mode.c_str(), states_[0].armed, states_[0].connected,
             kModelNames[1].c_str(), states_[1].mode.c_str(), states_[1].armed, states_[1].connected,
             kModelNames[2].c_str(), states_[2].mode.c_str(), states_[2].armed, states_[2].connected,
             kModelNames[3].c_str(), states_[3].mode.c_str(), states_[3].armed, states_[3].connected);
  }

  bool allReady() const
  {
    if (!model_received_)
    {
      return false;
    }

    for (int i = 0; i < kUavCount; ++i)
    {
      if (!states_[i].connected || !states_[i].armed || states_[i].mode != "OFFBOARD")
      {
        return false;
      }
    }
    return true;
  }

  bool statesConnected() const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (!states_[i].connected)
      {
        return false;
      }
    }
    return true;
  }

  bool localPosesReceived() const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (!local_pose_received_[i])
      {
        return false;
      }
    }
    return true;
  }

  void primeSetpoints(ros::Rate& rate)
  {
    ROS_INFO("[formation] priming OFFBOARD setpoint stream for 2 seconds");
    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < 2.0)
    {
      publishCurrentPoseSetpoints();
      rate.sleep();
    }
  }

  void streamFor(double seconds)
  {
    ros::Rate rate(kRateHz);
    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < seconds)
    {
      publishCurrentPoseSetpoints();
      rate.sleep();
    }
  }

  void publishCurrentPoseSetpoints()
  {
    std::array<geometry_msgs::PoseStamped, kUavCount> setpoints;
    for (int i = 0; i < kUavCount; ++i)
    {
      setpoints[i] = toPose(local_positions_[i].x, local_positions_[i].y,
                            local_positions_[i].z, local_yaws_[i]);
    }
    publishPoseSetpoints(setpoints);
  }

  void publishLocalHomePoseSetpoints()
  {
    std::array<geometry_msgs::PoseStamped, kUavCount> setpoints;
    for (int i = 0; i < kUavCount; ++i)
    {
      setpoints[i] = toPose(local_home_[i].x, local_home_[i].y,
                            local_home_[i].z, home_yaws_[i]);
    }
    publishPoseSetpoints(setpoints);
  }

  void publishLocalPoseTargets(const std::array<Vec3, kUavCount>& targets)
  {
    std::array<geometry_msgs::PoseStamped, kUavCount> setpoints;
    for (int i = 0; i < kUavCount; ++i)
    {
      setpoints[i] = toPose(targets[i].x, targets[i].y, targets[i].z, home_yaws_[i]);
    }
    publishPoseSetpoints(setpoints);
  }

  void publishWorldTargetsAsLocalPoseSetpoints(const std::array<Vec3, kUavCount>& world_targets)
  {
    std::array<Vec3, kUavCount> local_targets;
    for (int i = 0; i < kUavCount; ++i)
    {
      local_targets[i].x = local_home_[i].x + (world_targets[i].x - home_[i].x);
      local_targets[i].y = local_home_[i].y + (world_targets[i].y - home_[i].y);
      local_targets[i].z = kFlightZ;
    }
    publishLocalPoseTargets(local_targets);
  }

  void publishVelocityToTargets(const std::array<Vec3, kUavCount>& targets)
  {
    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;
    for (int i = 0; i < kUavCount; ++i)
    {
      const Vec3 err = opSub(targets[i], positions_[i]);
      Vec3 v = opMul({err.x, err.y, 0.0}, 0.8);
      const double speed = norm2d(v);
      if (speed > kMaxSpeed)
      {
        v = opMul(v, kMaxSpeed / speed);
      }

      const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
      setpoints[i] = toTwist(v.x, v.y, vz);
    }
    publishTwistSetpoints(setpoints);
  }

  // 恒速目标追踪: 在距目标 slow_radius 范围内线性减速, 避免冲过头。
  // 论文 1 m/s 动态条件使用此函数, 替代 P 控制的 publishVelocityToTargets。
  // 已集成 arrived_flags_ 到达锁存和 max_accel_xy 加速度限制。
  void publishConstantSpeedToTargets(
      const std::array<Vec3, kUavCount>& targets,
      double speed = -1.0,
      double slow_r = -1.0)
  {
    const double eff_speed = (speed > 0.0) ? speed : horizontal_speed_;
    const double eff_slow_r = (slow_r > 0.0) ? slow_r : slow_radius_;

    // 保存当前目标位置供 CSV 记录
    current_targets_ = targets;

    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;

    for (int i = 0; i < kUavCount; ++i)
    {
      // 到达锁存: 先到的 UAV 改为低速保持目标, 不再硬发零速度。
      // 避免 UAV 被锁死后漂离目标, 等齐其他 UAV 时位置误差已过大。
      if (arrived_flags_[i])
      {
        Vec3 v = settleVelocityToTarget(i, targets[i]);
        const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
        setpoints[i] = toTwist(v.x, v.y, vz);
        last_cmd_speed_xy_[i] = norm2d(v);
        continue;
      }

      const Vec3 err = opSub(targets[i], positions_[i]);
      const double dist_xy = norm2d(err);

      Vec3 v{0.0, 0.0, 0.0};

      if (dist_xy > 1.0e-3)
      {
        const Vec3 dir = normalize2d(err);
        double cmd_speed = eff_speed;

        if (dist_xy < eff_slow_r)
        {
          cmd_speed = eff_speed * dist_xy / eff_slow_r;
        }
        cmd_speed = std::min(cmd_speed, max_horizontal_speed_);

        v.x = dir.x * cmd_speed;
        v.y = dir.y * cmd_speed;
      }

      // 加速度限制后再记录真实发布速度到 CSV
      v = limitAccel(i, v);

      const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
      setpoints[i] = toTwist(v.x, v.y, vz);
      last_cmd_speed_xy_[i] = norm2d(v);
    }

    publishTwistSetpoints(setpoints);
  }

  // 非对称加速度限制: 将 desired_v 限制在当前 cmd_vel 的范围内。
  // 加速时使用 max_accel_xy_, 减速/反向时使用 max_decel_xy_ (更大)。
  // 避免接近目标时因加速限制导致命令速度无法快速降低, 造成过冲。
  Vec3 limitAccel(int i, const Vec3& desired_v)
  {
    const ros::Time now = ros::Time::now();

    // 每架 UAV 独立时间戳, 避免 for 循环内 4 机共享 dt 导致 iris_1/2/3 dt 被压缩
    double dt = last_cmd_time_[i].isZero()
                  ? 1.0 / kRateHz
                  : (now - last_cmd_time_[i]).toSec();

    dt = clamp(dt, 1.0 / 100.0, 0.2);

    Vec3 dv = opSub(desired_v, last_cmd_vel_[i]);
    const double dv_norm = norm2d(dv);

    // 判断当前是加速还是减速/反向:
    // 命令速度变小或方向反转 → 使用更大的减速限幅
    const double current_speed = norm2d(last_cmd_vel_[i]);
    const double desired_speed = norm2d(desired_v);
    const double dot =
        last_cmd_vel_[i].x * desired_v.x +
        last_cmd_vel_[i].y * desired_v.y;

    const bool braking = desired_speed < current_speed || dot < 0.0;
    const double accel_limit = braking ? max_decel_xy_ : max_accel_xy_;
    const double max_dv = accel_limit * dt;

    Vec3 out = desired_v;
    if (dv_norm > max_dv)
    {
      out = opAdd(last_cmd_vel_[i], opMul(dv, max_dv / dv_norm));
    }

    last_cmd_vel_[i] = out;
    last_cmd_time_[i] = now;
    return out;
  }

  double minPairwiseDistance(const std::array<Vec3, kUavCount>& pts) const
  {
    double min_d = std::numeric_limits<double>::max();
    for (int i = 0; i < kUavCount; ++i)
    {
      for (int j = i + 1; j < kUavCount; ++j)
      {
        const Vec3 d = opSub(pts[i], pts[j]);
        const double dd = norm2d(d);
        if (dd < min_d) min_d = dd;
      }
    }
    return min_d;
  }

  void publishZeroVelocity()
  {
    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;
    for (int i = 0; i < kUavCount; ++i)
    {
      setpoints[i] = toTwist(0.0, 0.0, 0.0);
      last_cmd_speed_xy_[i] = 0.0;
      last_cmd_vel_[i] = {0.0, 0.0, 0.0};
      last_cmd_time_[i] = ros::Time::now();
    }
    publishTwistSetpoints(setpoints);
  }

  // 安全距离检查: 任意两机水平距离 < min_pairwise_distance_ 时急停
  bool safetyStopIfTooClose(const std::string& stage_name)
  {
    const double d_min = minPairwiseDistance(positions_);
    if (d_min < min_pairwise_distance_)
    {
      ROS_WARN_THROTTLE(
          2.0,
          "[formation] %s safety stop: min_pairwise=%.2f < %.2f",
          stage_name.c_str(), d_min, min_pairwise_distance_);
      publishZeroVelocity();
      return true;
    }
    return false;
  }

  // 单机低速保持: 以 P 控制将单架 UAV 拉回目标, 限制最大速度 ≤ settle_speed_max_,
  // 经过 limitAccel 加速度限制。
  // 用于 arrived_flags_[i] == true 时的保持控制, 以及 SETTLE 阶段的全体修正。
  Vec3 settleVelocityToTarget(int i, const Vec3& target)
  {
    const Vec3 err = opSub(target, positions_[i]);
    Vec3 v = opMul({err.x, err.y, 0.0}, settle_p_gain_);

    const double speed_xy = norm2d(v);
    if (speed_xy > settle_speed_max_)
    {
      v = opMul(v, settle_speed_max_ / speed_xy);
    }

    return limitAccel(i, v);
  }

  // 从 Gazebo ModelStates twist 读取的真值速度
  double actualSpeedXY(int i) const
  {
    return norm2d(velocities_[i]);
  }

  // 全队低速修正控制 (SETTLE / arrived 锁存期间):
  // 调用 settleVelocityToTarget 逐机控制, 避免 UAV 被硬锁存后漂离目标。
  void publishSettleToTargets(const std::array<Vec3, kUavCount>& targets)
  {
    // 保存当前目标供 CSV 记录
    current_targets_ = targets;

    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;

    for (int i = 0; i < kUavCount; ++i)
    {
      Vec3 v = settleVelocityToTarget(i, targets[i]);

      const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
      setpoints[i] = toTwist(v.x, v.y, vz);
      last_cmd_speed_xy_[i] = norm2d(v);
    }

    publishTwistSetpoints(setpoints);
  }

  // 全队是否已在目标位置稳定:
  // target_dist_xy < settle_pos_tolerance_
  // 且 cmd_speed < settle_speed_tolerance_
  // 且实际真值速度 < settle_speed_tolerance_
  bool allStableAtTargets(const std::array<Vec3, kUavCount>& targets)
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      const double dist = norm2d(opSub(targets[i], positions_[i]));
      const double cmd_speed = norm2d(last_cmd_vel_[i]);
      const double actual_speed = actualSpeedXY(i);

      if (dist > settle_pos_tolerance_ ||
          cmd_speed > settle_speed_tolerance_ ||
          actual_speed > settle_speed_tolerance_)
      {
        return false;
      }
    }
    return true;
  }

  // 重置速度差分状态 (进入新动态阶段时调用)
  void resetSpeedDiagState()
  {
    has_prev_positions_ = false;
    last_speed_csv_stamp_ = ros::Time(0);
  }

  // 重置到达锁存状态 (进入新阶段时调用)
  void resetArrivedFlags()
  {
    arrived_flags_.fill(false);
  }

  // 更新到达锁存: 每架 UAV 距离目标 < tolerance 且命令速度 < arrive_speed_threshold_
  // 且实际真值速度 < arrive_speed_threshold_ 后才标记 arrived_flags_[i]=true。
  // 先到的 UAV 以 settleVelocityToTarget 低速保持等待其他 UAV。
  // 返回 true 表示全体到达。
  bool updateArrivedFlags(const std::array<Vec3, kUavCount>& targets,
                          double tolerance)
  {
    bool all_arrived = true;
    for (int i = 0; i < kUavCount; ++i)
    {
      const Vec3 err = opSub(targets[i], positions_[i]);
      const double cmd_speed = norm2d(last_cmd_vel_[i]);
      const double actual_speed = actualSpeedXY(i);
      // 位置误差 < tolerance 且命令速度和实际速度都降到接近零 → 标记到达
      if (norm2d(err) < tolerance &&
          cmd_speed < arrive_speed_threshold_ &&
          actual_speed < arrive_speed_threshold_)
      {
        arrived_flags_[i] = true;
      }
      all_arrived = all_arrived && arrived_flags_[i];
    }
    return all_arrived;
  }

  // 编队整体平移控制 (Translate 阶段专用):
  // 计算编队中心到目标中心的偏差决定整体速度,
  // 每架 UAV 速度 = 整体速度 + 小的队形修正速度。
  // 这样四架机更像一个刚体平移, 避免各自独立追点导致的不同步。
  void publishTranslateVelocity(const std::array<Vec3, kUavCount>& targets,
                                double speed = -1.0,
                                double slow_r = -1.0)
  {
    const double eff_speed = (speed > 0.0) ? speed : translate_speed_;
    const double eff_slow_r = (slow_r > 0.0) ? slow_r : slow_radius_;

    // 保存当前目标位置供 CSV 记录
    current_targets_ = targets;

    Vec3 current_center = formationCenter(positions_);
    Vec3 target_center = formationCenter(targets);

    Vec3 center_err = opSub(target_center, current_center);
    Vec3 center_dir = normalize2d(center_err);
    double center_dist = norm2d(center_err);

    // 编队中心速度: 距离远时巡航, 距离近时线性减速
    double center_speed = eff_speed;
    if (center_dist < eff_slow_r)
    {
      center_speed = eff_speed * center_dist / eff_slow_r;
    }
    center_speed = std::min(center_speed, max_horizontal_speed_);
    Vec3 v_center = opMul(center_dir, center_speed);

    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;

    for (int i = 0; i < kUavCount; ++i)
    {
      // 到达锁存: 改用 settleVelocityToTarget 低速保持, 不发零速度
      if (arrived_flags_[i])
      {
        Vec3 v = settleVelocityToTarget(i, targets[i]);
        const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
        setpoints[i] = toTwist(v.x, v.y, vz);
        last_cmd_speed_xy_[i] = norm2d(v);
        continue;
      }

      Vec3 shape_err = opSub(targets[i], positions_[i]);
      // 队形修正: 单机位置偏差乘以 translate_shape_gain_ 加到整体速度上
      // 使用较小增益避免左右两侧速度差过大导致 "不同频" 假象
      Vec3 v_shape = opMul({shape_err.x, shape_err.y, 0.0}, translate_shape_gain_);
      Vec3 v = opAdd(v_center, v_shape);

      // 整体速度上限使用 eff_speed, 而非 max_horizontal_speed_
      // 这样才能让 translate_speed=0.8 真正生效
      const double speed_limit = std::min(eff_speed, max_horizontal_speed_);
      if (norm2d(v) > speed_limit)
      {
        v = opMul(v, speed_limit / norm2d(v));
      }

      // 加速度限制后再记录真实发布速度到 CSV
      v = limitAccel(i, v);

      const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
      setpoints[i] = toTwist(v.x, v.y, vz);
      last_cmd_speed_xy_[i] = norm2d(v);
    }

    publishTwistSetpoints(setpoints);
  }

  // 制动感知速度规划:
  // 在恒速追点基础上加入 (a) 制动距离限制 和 (b) 实际速度阻尼,
  // 避免高速接近目标时因减速距离不足而过冲。
  Vec3 brakingAwareVelocityToTarget(
      int i,
      const Vec3& target,
      double max_speed,
      double slow_radius,
      double brake_accel)
  {
    const Vec3 err = opSub(target, positions_[i]);
    const double dist = norm2d(err);

    if (dist < 1.0e-3)
    {
      return {0.0, 0.0, 0.0};
    }

    const Vec3 dir = normalize2d(err);

    // 真值速度 (用于速度阻尼)
    const Vec3 actual_v{velocities_[i].x, velocities_[i].y, 0.0};

    double cmd_speed = max_speed;

    // 1. 原线性减速
    if (dist < slow_radius)
    {
      cmd_speed = max_speed * dist / slow_radius;
    }

    // 2. 制动距离限制: 当前速度下需要多远才能刹停
    const double stop_margin = stage_target_tolerance_;
    const double brake_dist = std::max(0.0, dist - stop_margin);
    const double brake_limited_speed =
        std::sqrt(std::max(0.0, 2.0 * brake_accel * brake_dist));

    cmd_speed = std::min(cmd_speed, brake_limited_speed);

    // 3. 速度阻尼: 在减速区内加入反向阻尼抑制实际速度
    Vec3 v_cmd = opMul(dir, cmd_speed);
    if (dist < slow_radius)
    {
      v_cmd.x -= velocity_damping_gain_ * actual_v.x;
      v_cmd.y -= velocity_damping_gain_ * actual_v.y;
    }

    // 4. 限幅
    const double s = norm2d(v_cmd);
    if (s > max_speed)
    {
      v_cmd = opMul(v_cmd, max_speed / s);
    }

    return v_cmd;
  }

  // 制动感知速度发布函数 (EXPAND_SHRINK RETURN 专用):
  // 使用 brakingAwareVelocityToTarget 逐机规划 + 非对称 limitAccel。
  void publishBrakeAwareSpeedToTargets(
      const std::array<Vec3, kUavCount>& targets,
      double speed,
      double slow_r,
      double brake_accel)
  {
    current_targets_ = targets;

    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;

    for (int i = 0; i < kUavCount; ++i)
    {
      Vec3 v;

      if (arrived_flags_[i])
      {
        v = settleVelocityToTarget(i, targets[i]);
      }
      else
      {
        v = brakingAwareVelocityToTarget(i, targets[i],
                                         speed, slow_r, brake_accel);
        v = limitAccel(i, v);
      }

      const double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
      setpoints[i] = toTwist(v.x, v.y, vz);
      last_cmd_speed_xy_[i] = norm2d(v);
    }

    publishTwistSetpoints(setpoints);
  }

  void publishPoseSetpoints(
      const std::array<geometry_msgs::PoseStamped, kUavCount>& setpoints)
  {
    {
      std::lock_guard<std::mutex> lock(setpoint_mutex_);
      cached_pose_setpoints_ = setpoints;
      cached_pose_active_ = true;
      cached_twist_active_ = false;
    }
    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(setpoints[i]);
    }
  }

  void publishTwistSetpoints(
      const std::array<geometry_msgs::TwistStamped, kUavCount>& setpoints)
  {
    {
      std::lock_guard<std::mutex> lock(setpoint_mutex_);
      cached_twist_setpoints_ = setpoints;
      cached_pose_active_ = false;
      cached_twist_active_ = true;
    }
    for (int i = 0; i < kUavCount; ++i)
    {
      twist_pubs_[i].publish(setpoints[i]);
    }
  }

  void setpointTimerCb(const ros::TimerEvent&)
  {
    std::array<geometry_msgs::PoseStamped, kUavCount> pose_setpoints;
    std::array<geometry_msgs::TwistStamped, kUavCount> twist_setpoints;
    bool publish_pose = false;
    bool publish_twist = false;
    {
      std::lock_guard<std::mutex> lock(setpoint_mutex_);
      publish_pose = cached_pose_active_;
      publish_twist = cached_twist_active_;
      pose_setpoints = cached_pose_setpoints_;
      twist_setpoints = cached_twist_setpoints_;
    }

    const ros::Time stamp = ros::Time::now();
    for (int i = 0; i < kUavCount; ++i)
    {
      if (publish_pose)
      {
        pose_setpoints[i].header.stamp = stamp;
        pose_pubs_[i].publish(pose_setpoints[i]);
      }
      else if (publish_twist)
      {
        twist_setpoints[i].header.stamp = stamp;
        twist_pubs_[i].publish(twist_setpoints[i]);
      }
    }
  }

  void enterStage(Stage next)
  {
    stage_ = next;
    stage_start_ = ros::Time::now();
    publishStage();

    switch (stage_)
    {
      case Stage::TAKEOFF:
        takeoff_start_z_ = local_positions_[0].z;
        for (int i = 0; i < kUavCount; ++i)
        {
          takeoff_start_z_ = std::min(takeoff_start_z_, local_positions_[i].z);
        }
        takeoff_target_z_ = takeoff_start_z_;
        takeoff_altitude_ready_since_ = ros::Time(0);
        ROS_INFO("[formation] Stage TAKEOFF: climb from %.2f m to %.1f m at %.2f m/s",
                 takeoff_start_z_, kFlightZ, takeoff_climb_rate_);
        break;
      case Stage::HOVER:
        ROS_INFO("[formation] Stage 1 Hovering: 2 seconds");
        break;
      case Stage::EXPAND_SHRINK:
        motion_subphase_ = MotionSubphase::OUTBOUND;
        resetSpeedDiagState();
        resetArrivedFlags();
        std::fill(last_cmd_vel_.begin(), last_cmd_vel_.end(), Vec3{0.0, 0.0, 0.0});
        for (int i = 0; i < kUavCount; ++i) last_cmd_time_[i] = ros::Time(0);
        ROS_INFO("[formation] Stage 2 Expanding & Shrinking: 2.0 m outward and back at %.2f m/s",
                 horizontal_speed_);
        break;
      case Stage::TRANSLATE:
        motion_subphase_ = MotionSubphase::OUTBOUND;
        resetSpeedDiagState();
        resetArrivedFlags();
        std::fill(last_cmd_vel_.begin(), last_cmd_vel_.end(), Vec3{0.0, 0.0, 0.0});
        for (int i = 0; i < kUavCount; ++i) last_cmd_time_[i] = ros::Time(0);
        ROS_INFO("[formation] Stage 3 Translating: +3.0 m ENU-Y and back at %.2f m/s",
                 translate_speed_);
        break;
      case Stage::CHASE_RESTORE:
        motion_subphase_ = MotionSubphase::OUTBOUND;
        resetSpeedDiagState();
        resetArrivedFlags();
        std::fill(last_cmd_vel_.begin(), last_cmd_vel_.end(), Vec3{0.0, 0.0, 0.0});
        for (int i = 0; i < kUavCount; ++i) last_cmd_time_[i] = ros::Time(0);
        captureChaseTargets();
        ROS_INFO("[formation] Stage 4 Chase/Restore: cyclic position swap and return at %.2f m/s",
                 chase_speed_);
        break;
      case Stage::LAND:
        landing_start_z_ = local_positions_[0].z;
        for (int i = 1; i < kUavCount; ++i)
        {
          landing_start_z_ = std::max(landing_start_z_, local_positions_[i].z);
        }
        landing_target_z_ = landing_start_z_;
        ROS_INFO("[formation] Landing: descend from %.2f m at %.2f m/s",
                 landing_start_z_, landing_descent_rate_);
        break;
      case Stage::DONE:
        last_disarm_request_ = ros::Time(0);
        ROS_INFO("[formation] ground altitude reached; holding landing setpoints until all UAVs disarm");
        break;
      default:
        break;
    }
  }

  void publishStage()
  {
    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(stage_);
    stage_pub_.publish(msg);
  }

  void runTakeoff()
  {
    const double elapsed = stageElapsed();
    publishTakeoffTargets();
    const bool altitude_ready = allAtAltitude(kFlightZ, kTakeoffAltitudeTolerance);

    if (altitude_ready)
    {
      if (takeoff_altitude_ready_since_.isZero())
      {
        takeoff_altitude_ready_since_ = ros::Time::now();
      }
    }
    else
    {
      takeoff_altitude_ready_since_ = ros::Time(0);
    }

    const double stable_time = takeoff_altitude_ready_since_.isZero()
                                 ? 0.0
                                 : (ros::Time::now() - takeoff_altitude_ready_since_).toSec();

    ROS_DEBUG_THROTTLE(2.0,
                      "[formation] TAKEOFF t=%.1f target_z=%.2f z=[%.2f %.2f %.2f %.2f] ready=%d stable=%.1f",
                      elapsed,
                      takeoff_target_z_,
                      positions_[0].z, positions_[1].z,
                      positions_[2].z, positions_[3].z, allReady(), stable_time);

    if (!allReady() && (ros::Time::now() - last_ready_retry_).toSec() > 1.0)
    {
      ROS_WARN("[formation] waiting for all vehicles to be armed and OFFBOARD before completing takeoff");
      reassertReadyForFlight();
      logVehicleStates();
      last_ready_retry_ = ros::Time::now();
    }

    if (altitude_ready && stable_time >= 1.0)
    {
      enterStage(Stage::HOVER);
      return;
    }

    if (elapsed > takeoff_timeout_)
    {
      ROS_WARN_THROTTLE(10.0,
                        "[formation] takeoff exceeds %.1f s; holding TAKEOFF until all UAVs reach %.1f m",
                        takeoff_timeout_, kFlightZ);
      if ((ros::Time::now() - last_ready_retry_).toSec() > 1.0)
      {
        reassertReadyForFlight();
        logVehicleStates();
        last_ready_retry_ = ros::Time::now();
      }
    }
  }

  void publishTakeoffTargets()
  {
    const double elapsed = stageElapsed();
    takeoff_target_z_ = std::min(kFlightZ, takeoff_start_z_ + takeoff_climb_rate_ * elapsed);

    std::array<Vec3, kUavCount> targets = local_home_;
    for (int i = 0; i < kUavCount; ++i)
    {
      targets[i].z = takeoff_target_z_;
    }
    publishLocalPoseTargets(targets);
  }

  void runHover()
  {
    const double elapsed = stageElapsed();
    publishLocalHomePoseSetpoints();
    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 1 Hovering t=%.1f/2.0", elapsed);
    if (elapsed >= 2.0)
    {
      enterStage(Stage::EXPAND_SHRINK);
    }
  }

  void runExpandShrink()
  {
    if (safetyStopIfTooClose("EXPAND_SHRINK"))
    {
      logSpeedDiag();
      return;
    }

    if (motion_subphase_ == MotionSubphase::OUTBOUND)
    {
      // 向 expand 目标运动 (使用独立参数 expand_speed_ / expand_slow_radius_)
      publishConstantSpeedToTargets(expand_targets_,
                                    expand_speed_,
                                    expand_slow_radius_);

      if (updateArrivedFlags(expand_targets_, stage_target_tolerance_))
      {
        // 全队到达 → 进入 OUTBOUND_SETTLE, 不发送 publishZeroVelocity(),
        // 让 publishSettleToTargets 平滑接管。
        resetSpeedDiagState();
        stable_since_ = ros::Time(0);
        motion_subphase_ = MotionSubphase::OUTBOUND_SETTLE;
        return;
      }
    }
    else if (motion_subphase_ == MotionSubphase::OUTBOUND_SETTLE)
    {
      // 低速修正到 expand_targets_, 短暂稳定后直接进入 RETURN (不掉头停顿)
      publishSettleToTargets(expand_targets_);

      if (allStableAtTargets(expand_targets_))
      {
        if (stable_since_.isZero())
          stable_since_ = ros::Time::now();

        if ((ros::Time::now() - stable_since_).toSec() >= turnaround_settle_time_)
        {
          resetArrivedFlags();
          stable_since_ = ros::Time(0);
          motion_subphase_ = MotionSubphase::RETURN;
          return;
        }
      }
      else
      {
        stable_since_ = ros::Time(0);
      }
    }
    else if (motion_subphase_ == MotionSubphase::RETURN)
    {
      // 向 home 收缩, 使用制动感知控制避免高速过冲
      publishBrakeAwareSpeedToTargets(home_,
                                      expand_return_speed_,
                                      expand_return_slow_radius_,
                                      expand_brake_accel_xy_);

      if (updateArrivedFlags(home_, stage_target_tolerance_) &&
          allAtAltitude(kFlightZ, kStageAltitudeTolerance))
      {
        // 全队回到 home → 进入 RETURN_SETTLE, 不发送 publishZeroVelocity()
        resetSpeedDiagState();
        stable_since_ = ros::Time(0);
        motion_subphase_ = MotionSubphase::RETURN_SETTLE;
        return;
      }
    }
    else  // RETURN_SETTLE
    {
      // 低速修正到 home_, 全队稳定后计时 1s 再进入 TRANSLATE
      publishSettleToTargets(home_);

      if (allStableAtTargets(home_))
      {
        if (stable_since_.isZero())
          stable_since_ = ros::Time::now();

        if ((ros::Time::now() - stable_since_).toSec() >= stage_settle_time_)
        {
          publishZeroVelocity();
          enterStage(Stage::TRANSLATE);
          stable_since_ = ros::Time(0);
          return;
        }
      }
      else
      {
        stable_since_ = ros::Time(0);
      }
    }

    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 2 Expand/Shrink subphase=%d t=%.1f",
                      static_cast<int>(motion_subphase_), stageElapsed());
    logSpeedDiag();
  }

  void runTranslate()
  {
    if (safetyStopIfTooClose("TRANSLATE"))
    {
      logSpeedDiag();
      return;
    }

    if (motion_subphase_ == MotionSubphase::OUTBOUND)
    {
      // 编队整体平移控制 (formation-center based), 四机更像刚体
      publishTranslateVelocity(translate_targets_);

      if (updateArrivedFlags(translate_targets_, stage_target_tolerance_))
      {
        // 全队到达 → 进入 OUTBOUND_SETTLE, 不发送 publishZeroVelocity(),
        // 让 publishSettleToTargets 平滑接管。
        resetSpeedDiagState();
        stable_since_ = ros::Time(0);
        motion_subphase_ = MotionSubphase::OUTBOUND_SETTLE;
        return;
      }
    }
    else if (motion_subphase_ == MotionSubphase::OUTBOUND_SETTLE)
    {
      // 低速修正到 translate_targets_, 短暂稳定后直接进入 RETURN (不掉头停顿)
      publishSettleToTargets(translate_targets_);

      if (allStableAtTargets(translate_targets_))
      {
        if (stable_since_.isZero())
          stable_since_ = ros::Time::now();

        if ((ros::Time::now() - stable_since_).toSec() >= turnaround_settle_time_)
        {
          resetArrivedFlags();
          stable_since_ = ros::Time(0);
          motion_subphase_ = MotionSubphase::RETURN;
          return;
        }
      }
      else
      {
        stable_since_ = ros::Time(0);
      }
    }
    else if (motion_subphase_ == MotionSubphase::RETURN)
    {
      // 返回也使用编队整体平移控制, 保持一致
      publishTranslateVelocity(home_, translate_speed_, slow_radius_);

      if (updateArrivedFlags(home_, stage_target_tolerance_) &&
          allAtAltitude(kFlightZ, kStageAltitudeTolerance))
      {
        // 全队返回 home → 进入 RETURN_SETTLE
        resetSpeedDiagState();
        stable_since_ = ros::Time(0);
        motion_subphase_ = MotionSubphase::RETURN_SETTLE;
        return;
      }
    }
    else  // RETURN_SETTLE
    {
      // 低速修正到 home_, 全队稳定后计时 1s 再进入 CHASE_RESTORE
      publishSettleToTargets(home_);

      if (allStableAtTargets(home_))
      {
        if (stable_since_.isZero())
          stable_since_ = ros::Time::now();

        if ((ros::Time::now() - stable_since_).toSec() >= stage_settle_time_)
        {
          publishZeroVelocity();
          enterStage(Stage::CHASE_RESTORE);
          stable_since_ = ros::Time(0);
          return;
        }
      }
      else
      {
        stable_since_ = ros::Time(0);
      }
    }

    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 3 Translate subphase=%d t=%.1f",
                      static_cast<int>(motion_subphase_), stageElapsed());
    logSpeedDiag();
  }

  void runChaseRestore()
  {
    if (safetyStopIfTooClose("CHASE_RESTORE"))
    {
      logSpeedDiag();
      return;
    }

    if (motion_subphase_ == MotionSubphase::OUTBOUND)
    {
      // Chase 阶段使用独立 chase_speed_ 和更保守的 chase_slow_radius_
      publishConstantSpeedToTargets(chase_swap_targets_,
                                    chase_speed_, chase_slow_radius_);

      if (updateArrivedFlags(chase_swap_targets_, stage_target_tolerance_))
      {
        // 全队到达交换目标 → 进入 OUTBOUND_SETTLE, 不发送 publishZeroVelocity()
        resetSpeedDiagState();
        stable_since_ = ros::Time(0);
        motion_subphase_ = MotionSubphase::OUTBOUND_SETTLE;
        return;
      }
    }
    else if (motion_subphase_ == MotionSubphase::OUTBOUND_SETTLE)
    {
      // 低速修正到 chase_swap_targets_, 短暂稳定后直接进入 RETURN (不掉头停顿)
      publishSettleToTargets(chase_swap_targets_);

      if (allStableAtTargets(chase_swap_targets_))
      {
        if (stable_since_.isZero())
          stable_since_ = ros::Time::now();

        if ((ros::Time::now() - stable_since_).toSec() >= turnaround_settle_time_)
        {
          resetArrivedFlags();
          stable_since_ = ros::Time(0);
          motion_subphase_ = MotionSubphase::RETURN;
          return;
        }
      }
      else
      {
        stable_since_ = ros::Time(0);
      }
    }
    else if (motion_subphase_ == MotionSubphase::RETURN)
    {
      // 返回阶段使用 chase_speed_ 和 chase_slow_radius_, 避免回到 1.0 m/s
      publishConstantSpeedToTargets(chase_start_positions_,
                                    chase_speed_, chase_slow_radius_);

      if (updateArrivedFlags(chase_start_positions_, stage_target_tolerance_) &&
          allAtAltitude(kFlightZ, kStageAltitudeTolerance))
      {
        // 全队返回初始位置 → 进入 RETURN_SETTLE
        resetSpeedDiagState();
        stable_since_ = ros::Time(0);
        motion_subphase_ = MotionSubphase::RETURN_SETTLE;
        return;
      }
    }
    else  // RETURN_SETTLE
    {
      // 低速修正到 chase_start_positions_, 全队稳定后计时 1s 再进入 LAND
      publishSettleToTargets(chase_start_positions_);

      if (allStableAtTargets(chase_start_positions_))
      {
        if (stable_since_.isZero())
          stable_since_ = ros::Time::now();

        if ((ros::Time::now() - stable_since_).toSec() >= stage_settle_time_)
        {
          publishZeroVelocity();
          enterStage(Stage::LAND);
          stable_since_ = ros::Time(0);
          return;
        }
      }
      else
      {
        stable_since_ = ros::Time(0);
      }
    }

    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 4 Chase/Restore subphase=%d t=%.1f",
                      static_cast<int>(motion_subphase_), stageElapsed());
    logSpeedDiag();
  }

  void runLand()
  {
    const double elapsed = stageElapsed();
    landing_target_z_ =
      std::max(0.0, landing_start_z_ - landing_descent_rate_ * elapsed);

    std::array<Vec3, kUavCount> targets = local_home_;
    for (int i = 0; i < kUavCount; ++i)
    {
      targets[i].z = landing_target_z_;
    }
    // 通过统一缓存接口发布，明确关闭上一阶段的 twist setpoint。
    // 否则30Hz缓存定时器会继续发布旧速度命令，与LAND位置命令冲突。
    publishLocalPoseTargets(targets);

    ROS_DEBUG_THROTTLE(2.0,
                      "[formation] LAND t=%.1f target_z=%.2f z=[%.2f %.2f %.2f %.2f]",
                      elapsed, landing_target_z_,
                      positions_[0].z, positions_[1].z,
                      positions_[2].z, positions_[3].z);

    if (allLanded() || elapsed > landing_timeout_)
    {
      if (elapsed > landing_timeout_)
      {
        ROS_WARN("[formation] landing timeout %.1f s, requesting disarm anyway",
                 landing_timeout_);
      }
      enterStage(Stage::DONE);
    }
  }

  void runDisarm()
  {
    std::array<Vec3, kUavCount> targets = local_home_;
    for (int i = 0; i < kUavCount; ++i)
    {
      targets[i].z = 0.0;
    }
    // Keep the OFFBOARD pose stream alive while PX4's landed detector settles.
    // Exiting immediately after the first disarm request can trigger
    // COM_OF_LOSS_T and make failsafe, rather than this controller, finish
    // the landing.
    publishLocalPoseTargets(targets);

    if (allDisarmed())
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    const double request_period = stageElapsed() < force_disarm_delay_ ? 0.5 : 1.0;
    if (last_disarm_request_.isZero() ||
        (now - last_disarm_request_).toSec() >= request_period)
    {
      if (stageElapsed() < force_disarm_delay_)
      {
        requestDisarmOnce();
      }
      else if (allLanded())
      {
        requestForceDisarmOnce();
      }
      last_disarm_request_ = now;
    }

    ROS_INFO_THROTTLE(2.0,
                      "[formation] waiting for PX4 landed detection/disarm, armed=[%d %d %d %d]",
                      armed_flags_[0].load(), armed_flags_[1].load(),
                      armed_flags_[2].load(), armed_flags_[3].load());
  }

  bool allAtAltitude(double z, double tolerance) const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (std::fabs(positions_[i].z - z) > tolerance)
      {
        return false;
      }
    }
    return true;
  }

  bool allAtTargets(const std::array<Vec3, kUavCount>& targets, double tolerance) const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      const Vec3 err = opSub(targets[i], positions_[i]);
      if (norm2d(err) > tolerance)
      {
        return false;
      }
    }
    return true;
  }

  bool allLanded() const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (positions_[i].z > 0.25)
      {
        return false;
      }
    }
    return true;
  }

  bool allDisarmed() const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (armed_flags_[i].load())
      {
        return false;
      }
    }
    return true;
  }

  void requestDisarmOnce()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (!armed_flags_[i].load())
      {
        continue;
      }

      mavros_msgs::CommandBool srv;
      srv.request.value = false;
      if (!arm_clients_[i].call(srv))
      {
        ROS_WARN_THROTTLE(2.0, "[formation] %s disarm service call failed",
                          kModelNames[i].c_str());
      }
    }
  }

  void requestForceDisarmOnce()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (!armed_flags_[i].load())
      {
        continue;
      }

      mavros_msgs::CommandLong srv;
      srv.request.broadcast = false;
      srv.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
      srv.request.confirmation = 0;
      srv.request.param1 = 0.0;
      srv.request.param2 = 21196.0;  // PX4 force-disarm magic value
      srv.request.param3 = 0.0;
      srv.request.param4 = 0.0;
      srv.request.param5 = 0.0;
      srv.request.param6 = 0.0;
      srv.request.param7 = 0.0;

      if (!command_clients_[i].call(srv) || !srv.response.success)
      {
        ROS_WARN_THROTTLE(2.0, "[formation] %s force-disarm command failed",
                          kModelNames[i].c_str());
      }
      else
      {
        ROS_WARN("[formation] %s force-disarm accepted after %.1f s on ground",
                 kModelNames[i].c_str(), stageElapsed());
      }
    }
  }

  void disarmAll()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      mavros_msgs::CommandBool srv;
      srv.request.value = false;

      for (int attempt = 1; attempt <= 5 && ros::ok(); ++attempt)
      {
        if (arm_clients_[i].call(srv) && srv.response.success)
        {
          ROS_DEBUG("[formation] %s disarmed", kModelNames[i].c_str());
          break;
        }
        ros::Duration(0.3).sleep();
      }
    }
  }

  // 速度诊断 CSV ─────────────────────────────────────────────
  void initSpeedCsv()
  {
    boost::filesystem::create_directories(speed_csv_dir_);
    const std::string path = speed_csv_dir_ + "/formation_speed_diag.csv";
    speed_csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
    if (!speed_csv_.is_open())
    {
      ROS_WARN("[formation] cannot open speed diag CSV: %s", path.c_str());
      return;
    }
    speed_csv_ << std::fixed << std::setprecision(6);
    speed_csv_ << "stamp,stage,subphase,"
               << "uav_id,"
               << "gt_x,gt_y,gt_z,"
               << "gt_vx,gt_vy,gt_vz,gt_speed_xy,"
               << "cmd_speed_xy,"
               << "target_dist_xy,arrived_flag,min_pairwise_distance,"
               << "target_x,target_y,target_z\n";
    ROS_INFO("[formation] speed diag CSV: %s", path.c_str());
  }

  void logSpeedDiag()
  {
    if (!speed_csv_.is_open())
      return;

    // 每 100ms (10 Hz) 写一次避免文件过大
    const ros::Time now = ros::Time::now();
    if (!last_speed_csv_stamp_.isZero() &&
        (now - last_speed_csv_stamp_).toSec() < 0.1)
      return;

    // 先计算 dt, 再更新 last_speed_csv_stamp_, 避免 dt ≈ 0 的 bug
    double dt = 0.0;
    if (has_prev_positions_ && !last_speed_csv_stamp_.isZero())
    {
      dt = (now - last_speed_csv_stamp_).toSec();
    }
    if (dt <= 1.0e-6)
      dt = 0.1;

    last_speed_csv_stamp_ = now;

    const int s = static_cast<int>(stage_);
    const int sp = static_cast<int>(motion_subphase_);
    const double stamp = now.toSec();

    // 当前时刻全队最小机间距 (每帧统一值, 写给所有 4 行)
    const double frame_min_pairwise = minPairwiseDistance(positions_);

    for (int i = 0; i < kUavCount; ++i)
    {
      double gt_vx = 0.0, gt_vy = 0.0, gt_speed_xy = 0.0;
      if (has_prev_positions_)
      {
        gt_vx = (positions_[i].x - prev_positions_[i].x) / dt;
        gt_vy = (positions_[i].y - prev_positions_[i].y) / dt;
        gt_speed_xy = std::sqrt(gt_vx * gt_vx + gt_vy * gt_vy);
      }

      const Vec3 err = opSub(current_targets_[i], positions_[i]);
      const double td_xy = norm2d(err);

      speed_csv_ << stamp << ','
                 << s << ',' << sp << ','
                 << i << ','
                 << positions_[i].x << ','
                 << positions_[i].y << ','
                 << positions_[i].z << ','
                 << gt_vx << ',' << gt_vy << ',' << 0.0 << ','
                 << gt_speed_xy << ','
                 << last_cmd_speed_xy_[i] << ','
                 << td_xy << ','
                 << (arrived_flags_[i] ? 1 : 0) << ','
                 << frame_min_pairwise << ','
                 << current_targets_[i].x << ','
                 << current_targets_[i].y << ','
                 << current_targets_[i].z << '\n';
    }
    speed_csv_.flush();

    // 保存前帧位置用于下一帧差分
    for (int i = 0; i < kUavCount; ++i)
      prev_positions_[i] = positions_[i];
    has_prev_positions_ = true;
  }

  Vec3 formationCenter(const std::array<Vec3, kUavCount>& points) const
  {
    Vec3 center{0.0, 0.0, 0.0};
    for (int i = 0; i < kUavCount; ++i)
    {
      center = opAdd(center, points[i]);
    }
    return opMul(center, 1.0 / static_cast<double>(kUavCount));
  }

  void buildMotionTargets()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      const Vec3 radial = normalize2d(opSub(home_[i], center_));
      expand_targets_[i] = opAdd(home_[i], opMul(radial, 2.0));
      expand_targets_[i].z = kFlightZ;

      translate_targets_[i] = opAdd(home_[i], {0.0, 3.0, 0.0});
      translate_targets_[i].z = kFlightZ;
    }
  }

  std::array<Vec3, kUavCount> interpolateTargets(
      const std::array<Vec3, kUavCount>& from,
      const std::array<Vec3, kUavCount>& to,
      double alpha) const
  {
    std::array<Vec3, kUavCount> targets;
    for (int i = 0; i < kUavCount; ++i)
    {
      targets[i] = {
        from[i].x + (to[i].x - from[i].x) * alpha,
        from[i].y + (to[i].y - from[i].y) * alpha,
        kFlightZ
      };
    }
    return targets;
  }

  void captureChaseTargets()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      chase_start_positions_[i] = positions_[i];
      chase_start_positions_[i].z = kFlightZ;
    }

    chase_swap_targets_[0] = chase_start_positions_[1];
    chase_swap_targets_[1] = chase_start_positions_[2];
    chase_swap_targets_[2] = chase_start_positions_[3];
    chase_swap_targets_[3] = chase_start_positions_[0];

    ROS_INFO("[formation] chase targets: iris_0 (%.2f,%.2f)->(%.2f,%.2f), "
             "iris_1 (%.2f,%.2f)->(%.2f,%.2f), "
             "iris_2 (%.2f,%.2f)->(%.2f,%.2f), "
             "iris_3 (%.2f,%.2f)->(%.2f,%.2f)",
             chase_start_positions_[0].x, chase_start_positions_[0].y,
             chase_swap_targets_[0].x, chase_swap_targets_[0].y,
             chase_start_positions_[1].x, chase_start_positions_[1].y,
             chase_swap_targets_[1].x, chase_swap_targets_[1].y,
             chase_start_positions_[2].x, chase_start_positions_[2].y,
             chase_swap_targets_[2].x, chase_swap_targets_[2].y,
             chase_start_positions_[3].x, chase_start_positions_[3].y,
             chase_swap_targets_[3].x, chase_swap_targets_[3].y);
  }

  double stageElapsed() const
  {
    return (ros::Time::now() - stage_start_).toSec();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber gazebo_sub_;
  ros::Publisher stage_pub_;
  std::array<ros::Subscriber, kUavCount> state_subs_;
  std::array<ros::Subscriber, kUavCount> local_pose_subs_;
  std::array<ros::Publisher, kUavCount> pose_pubs_;
  std::array<ros::Publisher, kUavCount> twist_pubs_;
  std::array<ros::ServiceClient, kUavCount> arm_clients_;
  std::array<ros::ServiceClient, kUavCount> command_clients_;
  std::array<ros::ServiceClient, kUavCount> mode_clients_;
  std::array<ros::ServiceClient, kUavCount> param_clients_;
  ros::Timer setpoint_timer_;

  std::array<Vec3, kUavCount> positions_;
  // Gazebo 真值速度 (从 ModelStates twist 读取, 用于稳定判断)
  std::array<Vec3, kUavCount> velocities_{};
  std::array<Vec3, kUavCount> local_positions_;
  std::array<Vec3, kUavCount> home_;
  std::array<Vec3, kUavCount> local_home_;
  std::array<Vec3, kUavCount> expand_targets_;
  std::array<Vec3, kUavCount> translate_targets_;
  std::array<Vec3, kUavCount> chase_start_positions_;
  std::array<Vec3, kUavCount> chase_swap_targets_;
  std::array<mavros_msgs::State, kUavCount> states_;
  std::array<std::atomic<bool>, kUavCount> armed_flags_;
  std::array<double, kUavCount> local_yaws_;
  std::array<double, kUavCount> home_yaws_;
  std::array<bool, kUavCount> local_pose_received_;
  std::array<geometry_msgs::PoseStamped, kUavCount> cached_pose_setpoints_;
  std::array<geometry_msgs::TwistStamped, kUavCount> cached_twist_setpoints_;
  std::mutex setpoint_mutex_;

  bool model_received_;
  bool cached_pose_active_;
  bool cached_twist_active_;
  Stage stage_;
  MotionSubphase motion_subphase_;
  ros::Time stage_start_;
  ros::Time takeoff_altitude_ready_since_;
  ros::Time last_ready_retry_;
  ros::Time last_disarm_request_;
  Vec3 center_;
  double landing_target_z_;
  double landing_start_z_;
  double landing_descent_rate_;
  double force_disarm_delay_;
  double takeoff_target_z_;
  double takeoff_start_z_;
  double takeoff_climb_rate_;
  double takeoff_timeout_;
  double landing_timeout_;
  // 1 m/s 动态运动参数
  double horizontal_speed_ = 1.0;
  double max_horizontal_speed_ = 1.0;
  double slow_radius_ = 0.80;
  double stage_target_tolerance_ = 0.25;
  double min_pairwise_distance_ = 0.8;
  // 阶段缓冲参数
  double phase_settle_time_ = 1.0;
  double stage_settle_time_ = 1.0;
  // 掉头缓冲: OUTBOUND→RETURN 子阶段内的短暂停顿 (默认 0.2s, 可设为 0 直接连续)
  double turnaround_settle_time_ = 0.2;
  ros::Time settle_start_;
  // 到达锁存
  std::array<bool, kUavCount> arrived_flags_{};
  // 当前阶段目标位置 (供 CSV 记录)
  std::array<Vec3, kUavCount> current_targets_{};
  // 加速度限制
  double max_accel_xy_ = 0.6;
  // 非对称减速限制: 允许快速制动, 避免低速距离内速度降不下来
  double max_decel_xy_ = 1.8;
  std::array<Vec3, kUavCount> last_cmd_vel_{};
  std::array<ros::Time, kUavCount> last_cmd_time_{};
  // EXPAND_SHRINK 独立参数 (避免全局 slow_radius 影响 TRANSLATE/CHASE)
  double expand_speed_ = 0.8;
  double expand_return_speed_ = 0.7;
  double expand_slow_radius_ = 1.4;
  double expand_return_slow_radius_ = 1.6;
  double expand_brake_accel_xy_ = 0.8;
  // 速度阻尼: 接近目标时抑制实际速度, 减少过冲
  double velocity_damping_gain_ = 0.6;
  // 各阶段独立速度参数 (可覆盖默认 horizontal_speed_)
  double translate_speed_ = 0.8;
  double chase_speed_ = 0.6;
  double chase_slow_radius_ = 1.0;
  // 到达判定: 位置误差 < tolerance 且命令速度 < 此阈值才标记 arrived
  double arrive_speed_threshold_ = 0.20;
  // 低速保持控制 (SETTLE / arrived 锁存期间)
  double settle_p_gain_ = 0.4;
  double settle_speed_max_ = 0.25;
  double settle_pos_tolerance_ = 0.25;
  double settle_speed_tolerance_ = 0.20;
  // SETTLE 稳定计时 (allStableAtTargets 持续为 true 后开始计时)
  ros::Time stable_since_;
  // Translate 整体平移控制
  double translate_shape_gain_ = 0.15;
  std::string speed_csv_dir_;
  std::array<double, kUavCount> last_cmd_speed_xy_{};
  std::ofstream speed_csv_;
  Vec3 prev_positions_[kUavCount]{};
  bool has_prev_positions_ = false;
  ros::Time last_speed_csv_stamp_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xtdrone_four_uav_formation");
  XtdroneFormationController controller;
  controller.spin();
  return 0;
}
