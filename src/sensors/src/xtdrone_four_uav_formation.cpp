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
#include <mutex>
#include <string>

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

  void publishZeroVelocity()
  {
    std::array<geometry_msgs::TwistStamped, kUavCount> setpoints;
    for (int i = 0; i < kUavCount; ++i)
    {
      setpoints[i] = toTwist(0.0, 0.0, 0.0);
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
        ROS_INFO("[formation] Stage 2 Expanding & Shrinking: 2.0 m outward and back");
        break;
      case Stage::TRANSLATE:
        ROS_INFO("[formation] Stage 3 Translating: +3.0 m ENU-Y and back");
        break;
      case Stage::CHASE_RESTORE:
        captureChaseTargets();
        ROS_INFO("[formation] Stage 4 Chase/Restore: cyclic position swap and return");
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
    const double elapsed = stageElapsed();
    std::array<Vec3, kUavCount> targets;
    if (elapsed < 5.0)
    {
      targets = interpolateTargets(home_, expand_targets_, clamp(elapsed / 5.0, 0.0, 1.0));
    }
    else
    {
      targets = interpolateTargets(expand_targets_, home_, clamp((elapsed - 5.0) / 5.0, 0.0, 1.0));
    }

    publishVelocityToTargets(targets);
    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 2 Expand/Shrink t=%.1f/10.0", elapsed);

    if (elapsed >= 10.0 && allAtTargets(home_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::TRANSLATE);
    }
  }

  void runTranslate()
  {
    const double elapsed = stageElapsed();
    std::array<Vec3, kUavCount> targets;
    if (elapsed < 5.0)
    {
      targets = interpolateTargets(home_, translate_targets_, clamp(elapsed / 5.0, 0.0, 1.0));
    }
    else
    {
      targets = interpolateTargets(translate_targets_, home_, clamp((elapsed - 5.0) / 5.0, 0.0, 1.0));
    }

    publishVelocityToTargets(targets);
    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 3 Translating t=%.1f/10.0", elapsed);

    if (elapsed >= 10.0 && allAtTargets(home_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::CHASE_RESTORE);
    }
  }

  void runChaseRestore()
  {
    const double elapsed = stageElapsed();
    std::array<Vec3, kUavCount> targets;
    if (elapsed < 5.0)
    {
      targets = interpolateTargets(chase_start_positions_, chase_swap_targets_,
                                   clamp(elapsed / 5.0, 0.0, 1.0));
    }
    else
    {
      targets = interpolateTargets(chase_swap_targets_, chase_start_positions_,
                                   clamp((elapsed - 5.0) / 5.0, 0.0, 1.0));
    }

    publishWorldTargetsAsLocalPoseSetpoints(targets);
    ROS_DEBUG_THROTTLE(2.0, "[formation] Stage 4 Chase/Restore t=%.1f/10.0", elapsed);

    if (elapsed >= 10.0 && allAtTargets(chase_start_positions_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::LAND);
    }
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xtdrone_four_uav_formation");
  XtdroneFormationController controller;
  controller.spin();
  return 0;
}
