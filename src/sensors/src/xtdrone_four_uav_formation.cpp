/**
 * @file  xtdrone_four_uav_formation.cpp
 * @brief XTDrone 四机编队飞行控制器 (Section 5.2)
 *
 * =========================== 功能描述 ===========================
 * 实现面向 XTDrone 仿真平台的 4 机编队飞行控制器。包含 7 阶段状态机：
 * WAIT → TAKEOFF → HOVER → EXPAND_SHRINK → TRANSLATE → ROTATE → LAND → DONE。
 * 支持起飞稳定等待、编队扩张-收缩、平移和旋转等多模态编队机动，
 * 以及自动重试 OFFBOARD/ARM 的容错机制。
 *
 * =========================== 数据流 ===========================
 * 订阅:
 *   /gazebo/model_states           (gazebo_msgs/ModelStates)
 *     - 获取所有无人机 Gazebo 真值位置，用于闭环控制
 *   /iris_X/mavros/state           (mavros_msgs/State)
 *     - 监控各无人机连接、解锁、模式状态
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
 *   TRANSLATE        — 沿 X 正向平移 3.0m 再返回 (5s + 5s)
 *   ROTATE           — 绕编队中心顺时针旋转 90° 再返回 (5s + 5s)
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
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr double kYaw = 0.0;
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
}  // namespace

class XtdroneFormationController
{
public:
  XtdroneFormationController()
    : nh_(),
      private_nh_("~"),
      model_received_(false),
      stage_(Stage::WAIT_FOR_GAZEBO),
      landing_target_z_(kFlightZ),
      takeoff_altitude_ready_since_(0.0),
      last_ready_retry_(0.0)
  {
    gazebo_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                                &XtdroneFormationController::gazeboCb, this);

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
      arm_clients_[i] = nh_.serviceClient<mavros_msgs::CommandBool>(
        mavros_ns + "/cmd/arming");
      mode_clients_[i] = nh_.serviceClient<mavros_msgs::SetMode>(
        mavros_ns + "/set_mode");
    }

    private_nh_.param("takeoff_timeout", takeoff_timeout_, 12.0);
    private_nh_.param("landing_timeout", landing_timeout_, 20.0);
  }

  void spin()
  {
    ros::Rate rate(kRateHz);

    ROS_INFO("[formation] waiting for /gazebo/model_states and MAVROS states");
    while (ros::ok() && (!model_received_ || !statesConnected()))
    {
      ros::spinOnce();
      ROS_INFO_THROTTLE(2.0, "[formation] model_received=%d connected=%d",
                        model_received_, statesConnected());
      rate.sleep();
    }

    for (int i = 0; i < kUavCount; ++i)
    {
      home_[i] = positions_[i];
      home_[i].z = kFlightZ;
    }
    center_ = formationCenter(home_);
    buildMotionTargets();

    ROS_INFO("[formation] home positions captured, center=(%.2f, %.2f)",
             center_.x, center_.y);

    primeSetpoints(rate);

    ROS_INFO("[formation] requesting OFFBOARD mode and arming");
    for (int i = 0; i < kUavCount && ros::ok(); ++i)
    {
      setMode(i);
      arm(i);
    }

    waitForAllReady(rate, 8.0);
    if (!allReady())
    {
      ROS_WARN("[formation] not all vehicles report armed OFFBOARD yet; takeoff will keep retrying");
      logVehicleStates();
    }

    enterStage(Stage::TAKEOFF);

    while (ros::ok())
    {
      ros::spinOnce();

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
        case Stage::ROTATE:
          runRotate();
          break;
        case Stage::LAND:
          runLand();
          break;
        case Stage::DONE:
          disarmAll();
          ROS_INFO("[formation] mission complete, landed and disarm requested");
          return;
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
    ROTATE,
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

  bool setMode(int i)
  {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = "OFFBOARD";

    for (int attempt = 1; attempt <= 5 && ros::ok(); ++attempt)
    {
      publishCurrentPoseSetpoints();
      if (mode_clients_[i].call(srv) && srv.response.mode_sent)
      {
        ROS_INFO("[formation] %s OFFBOARD request accepted on attempt %d",
                 kModelNames[i].c_str(), attempt);
        streamFor(0.3);
        return true;
      }
      ROS_WARN("[formation] %s OFFBOARD request failed on attempt %d",
               kModelNames[i].c_str(), attempt);
      streamFor(0.3);
    }
    return false;
  }

  bool arm(int i)
  {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;

    for (int attempt = 1; attempt <= 5 && ros::ok(); ++attempt)
    {
      publishCurrentPoseSetpoints();
      if (arm_clients_[i].call(srv) && srv.response.success)
      {
        ROS_INFO("[formation] %s armed on attempt %d",
                 kModelNames[i].c_str(), attempt);
        streamFor(0.3);
        return true;
      }
      ROS_WARN("[formation] %s arm failed on attempt %d",
               kModelNames[i].c_str(), attempt);
      streamFor(0.3);
    }
    return false;
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

  void waitForAllReady(ros::Rate& rate, double timeout)
  {
    const ros::Time start = ros::Time::now();
    ros::Time last_retry = ros::Time(0);

    while (ros::ok() && !allReady() && (ros::Time::now() - start).toSec() < timeout)
    {
      ros::spinOnce();
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

  void reassertReadyForFlight()
  {
    publishPoseTargets(home_);

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
          ROS_INFO("[formation] %s OFFBOARD re-request accepted", kModelNames[i].c_str());
        }
        else
        {
          ROS_WARN("[formation] %s OFFBOARD re-request failed", kModelNames[i].c_str());
        }
      }

      if (!states_[i].armed)
      {
        if (requestArmOnce(i))
        {
          ROS_INFO("[formation] %s arm re-request accepted", kModelNames[i].c_str());
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
    ROS_INFO("[formation] states: %s[%s armed=%d conn=%d] %s[%s armed=%d conn=%d] %s[%s armed=%d conn=%d] %s[%s armed=%d conn=%d]",
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

  void primeSetpoints(ros::Rate& rate)
  {
    ROS_INFO("[formation] priming OFFBOARD setpoint stream for 2 seconds");
    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < 2.0)
    {
      ros::spinOnce();
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
      ros::spinOnce();
      publishCurrentPoseSetpoints();
      rate.sleep();
    }
  }

  void publishCurrentPoseSetpoints()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(positions_[i].x, positions_[i].y,
                                   positions_[i].z, kYaw));
    }
  }

  void publishHomePoseSetpoints()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(home_[i].x, home_[i].y, home_[i].z, kYaw));
    }
  }

  void publishPoseTargets(const std::array<Vec3, kUavCount>& targets)
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(targets[i].x, targets[i].y, targets[i].z, kYaw));
    }
  }

  void publishVelocityToTargets(const std::array<Vec3, kUavCount>& targets)
  {
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
      twist_pubs_[i].publish(toTwist(v.x, v.y, vz));
    }
  }

  void publishZeroVelocity()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      twist_pubs_[i].publish(toTwist(0.0, 0.0, 0.0));
    }
  }

  void enterStage(Stage next)
  {
    stage_ = next;
    stage_start_ = ros::Time::now();

    switch (stage_)
    {
      case Stage::TAKEOFF:
        takeoff_altitude_ready_since_ = ros::Time(0);
        ROS_INFO("[formation] Stage TAKEOFF: climb to %.1f m", kFlightZ);
        break;
      case Stage::HOVER:
        ROS_INFO("[formation] Stage 1 Hovering: 2 seconds");
        break;
      case Stage::EXPAND_SHRINK:
        ROS_INFO("[formation] Stage 2 Expanding & Shrinking: 2.0 m outward and back");
        break;
      case Stage::TRANSLATE:
        ROS_INFO("[formation] Stage 3 Translating: +3.0 m X and back");
        break;
      case Stage::ROTATE:
        ROS_INFO("[formation] Stage 4 Rotating: clockwise 90 deg and back");
        break;
      case Stage::LAND:
        landing_target_z_ = kFlightZ;
        ROS_INFO("[formation] Landing: descending gradually");
        break;
      default:
        break;
    }
  }

  void runTakeoff()
  {
    const double elapsed = stageElapsed();
    publishPoseTargets(home_);
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

    ROS_INFO_THROTTLE(1.0,
                      "[formation] TAKEOFF t=%.1f z=[%.2f %.2f %.2f %.2f] ready=%d stable=%.1f",
                      elapsed,
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
      ROS_WARN_THROTTLE(2.0,
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

  void runHover()
  {
    const double elapsed = stageElapsed();
    publishPoseTargets(home_);
    ROS_INFO_THROTTLE(1.0, "[formation] Stage 1 Hovering t=%.1f/2.0", elapsed);
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
      targets = expand_targets_;
    }
    else
    {
      targets = home_;
    }

    publishVelocityToTargets(targets);
    ROS_INFO_THROTTLE(1.0, "[formation] Stage 2 Expand/Shrink t=%.1f/10.0", elapsed);

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
      targets = translate_targets_;
    }
    else
    {
      targets = home_;
    }

    publishVelocityToTargets(targets);
    ROS_INFO_THROTTLE(1.0, "[formation] Stage 3 Translating t=%.1f/10.0", elapsed);

    if (elapsed >= 10.0 && allAtTargets(home_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::ROTATE);
    }
  }

  void runRotate()
  {
    const double elapsed = stageElapsed();
    const double omega = 0.314;
    double angle = 0.0;

    if (elapsed < 5.0)
    {
      angle = -omega * elapsed;
    }
    else
    {
      angle = -omega * 5.0 + omega * (elapsed - 5.0);
    }
    angle = clamp(angle, -M_PI_2, 0.0);

    std::array<Vec3, kUavCount> targets;
    for (int i = 0; i < kUavCount; ++i)
    {
      const Vec3 rel = opSub(home_[i], center_);
      const double c = std::cos(angle);
      const double s = std::sin(angle);
      targets[i] = {
        center_.x + c * rel.x - s * rel.y,
        center_.y + s * rel.x + c * rel.y,
        kFlightZ
      };
    }

    publishVelocityToTargets(targets);
    ROS_INFO_THROTTLE(1.0,
                      "[formation] Stage 4 Rotating t=%.1f/10.0 angle=%.2f rad",
                      elapsed, angle);

    if (elapsed >= 10.0 && allAtTargets(home_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::LAND);
    }
  }

  void runLand()
  {
    const double elapsed = stageElapsed();
    const double dt = 1.0 / kRateHz;
    landing_target_z_ = std::max(0.0, landing_target_z_ - 0.35 * dt);

    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(home_[i].x, home_[i].y, landing_target_z_, kYaw));
    }

    ROS_INFO_THROTTLE(1.0,
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
          ROS_INFO("[formation] %s disarmed", kModelNames[i].c_str());
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

      translate_targets_[i] = opAdd(home_[i], {3.0, 0.0, 0.0});
      translate_targets_[i].z = kFlightZ;
    }
  }

  double stageElapsed() const
  {
    return (ros::Time::now() - stage_start_).toSec();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber gazebo_sub_;
  std::array<ros::Subscriber, kUavCount> state_subs_;
  std::array<ros::Publisher, kUavCount> pose_pubs_;
  std::array<ros::Publisher, kUavCount> twist_pubs_;
  std::array<ros::ServiceClient, kUavCount> arm_clients_;
  std::array<ros::ServiceClient, kUavCount> mode_clients_;

  std::array<Vec3, kUavCount> positions_;
  std::array<Vec3, kUavCount> home_;
  std::array<Vec3, kUavCount> expand_targets_;
  std::array<Vec3, kUavCount> translate_targets_;
  std::array<mavros_msgs::State, kUavCount> states_;

  bool model_received_;
  Stage stage_;
  ros::Time stage_start_;
  ros::Time takeoff_altitude_ready_since_;
  ros::Time last_ready_retry_;
  Vec3 center_;
  double landing_target_z_;
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
