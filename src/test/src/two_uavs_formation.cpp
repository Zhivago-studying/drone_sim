/**
 * @file  two_uavs_formation.cpp
 * @brief 两机编队飞行控制器 (iris_0 + iris_1)
 *
 * =========================== 5 阶段状态机 ===========================
 * WAIT → TAKEOFF → HOVER → EXPAND_SHRINK → ROTATE → LAND → DONE
 *
 *   1. TAKEOFF       — 爬升至 3m，稳定后推进
 *   2. HOVER         — 悬停 3s
 *   3. EXPAND_SHRINK — 沿径向扩展 2.0m 再收缩回原位 (5s + 5s)
 *   4. ROTATE        — 以两机中心为原点沿圆弧旋转 180° 再返回 (5s + 5s)
 *   5. LAND          — 着陆并上锁退出
 *
 * =========================== 启动方式 ===========================
 *   rosrun test two_uavs_formation
 *   或通过 launch 文件启动
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
  Vec3() = default;
  Vec3(double x_in, double y_in, double z_in)
    : x(x_in), y(y_in), z(z_in) {}

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 opAdd(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 opSub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 opMul(const Vec3& a, double s)      { return {a.x * s, a.y * s, a.z * s}; }

double length(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

double norm2d(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

Vec3 normalize2d(const Vec3& v)
{
  double n = norm2d(v);
  if (n < 1e-6) return {0.0, 0.0, 0.0};
  return {v.x / n, v.y / n, 0.0};
}

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

Vec3 rotate2d(const Vec3& p, const Vec3& center, double angle_rad)
{
  double dx = p.x - center.x;
  double dy = p.y - center.y;
  double c = std::cos(angle_rad);
  double s = std::sin(angle_rad);
  return {center.x + dx * c - dy * s,
          center.y + dx * s + dy * c,
          p.z};
}

double yawFromPose(const geometry_msgs::PoseStamped& pose)
{
  const auto& q = pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

namespace
{
constexpr int kUavCount = 2;
constexpr double kRateHz = 20.0;
constexpr double kFlightZ = 3.0;
constexpr double kMaxSpeed = 1.0;
constexpr double kTakeoffAltitudeTolerance = 0.15;
constexpr double kStageAltitudeTolerance = 0.25;
constexpr double kExpandDistance = 2.0;
constexpr double kRotateAngle = M_PI;  // 180度

const std::array<std::string, kUavCount> kModelNames = {"iris_0", "iris_1"};
}  // namespace

class TwoUavsFormation
{
public:
  TwoUavsFormation()
    : nh_(), private_nh_("~"),
      model_received_(false),
      stage_(Stage::WAIT_FOR_GAZEBO),
      landing_target_z_(kFlightZ),
      takeoff_target_z_(kFlightZ),
      takeoff_altitude_ready_since_(0.0),
      last_ready_retry_(0.0)
  {
    local_pose_received_.fill(false);
    local_yaws_.fill(0.0);
    home_yaws_.fill(0.0);

    gazebo_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                                &TwoUavsFormation::gazeboCb, this);

    for (int i = 0; i < kUavCount; ++i)
    {
      const std::string mavros_ns = "/" + kModelNames[i] + "/mavros";
      pose_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
        mavros_ns + "/setpoint_position/local", 10);
      twist_pubs_[i] = nh_.advertise<geometry_msgs::TwistStamped>(
        mavros_ns + "/setpoint_velocity/cmd_vel", 10);
      state_subs_[i] = nh_.subscribe<mavros_msgs::State>(
        mavros_ns + "/state", 10,
        boost::bind(&TwoUavsFormation::stateCb, this, _1, i));
      local_pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(
        mavros_ns + "/local_position/pose", 10,
        boost::bind(&TwoUavsFormation::localPoseCb, this, _1, i));
      arm_clients_[i] = nh_.serviceClient<mavros_msgs::CommandBool>(
        mavros_ns + "/cmd/arming");
      mode_clients_[i] = nh_.serviceClient<mavros_msgs::SetMode>(
        mavros_ns + "/set_mode");
    }

    private_nh_.param("takeoff_timeout", takeoff_timeout_, 12.0);
    private_nh_.param("landing_timeout", landing_timeout_, 20.0);
    private_nh_.param("takeoff_climb_rate", takeoff_climb_rate_, 0.45);
  }

  void spin()
  {
    ros::Rate rate(kRateHz);

    ROS_INFO("[formation2] waiting for /gazebo/model_states, MAVROS states, and local poses");
    while (ros::ok() && (!model_received_ || !statesConnected() || !localPosesReceived()))
    {
      ros::spinOnce();
      ROS_INFO_THROTTLE(2.0, "[formation2] model_received=%d connected=%d local_pose=%d",
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

    ROS_INFO("[formation2] home positions: iris_0=(%.2f, %.2f) iris_1=(%.2f, %.2f) center=(%.2f, %.2f)",
             home_[0].x, home_[0].y, home_[1].x, home_[1].y, center_.x, center_.y);

    primeSetpoints(rate);

    ROS_INFO("[formation2] requesting OFFBOARD mode and arming");
    for (int i = 0; i < kUavCount && ros::ok(); ++i)
      setMode(i);
    streamFor(0.5);

    for (int i = 0; i < kUavCount && ros::ok(); ++i)
      arm(i);

    waitForAllReady(rate, 8.0);
    if (!allReady())
    {
      ROS_WARN("[formation2] not all vehicles ready; takeoff will keep retrying");
    }

    enterStage(Stage::TAKEOFF);

    while (ros::ok())
    {
      ros::spinOnce();

      switch (stage_)
      {
        case Stage::TAKEOFF:       runTakeoff();       break;
        case Stage::HOVER:         runHover();         break;
        case Stage::EXPAND_SHRINK: runExpandShrink();  break;
        case Stage::ROTATE:        runRotate();        break;
        case Stage::LAND:          runLand();          break;
        case Stage::DONE:
          disarmAll();
          ROS_INFO("[formation2] mission complete");
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
    ROTATE,
    LAND,
    DONE
  };

  void gazeboCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    std::array<bool, kUavCount> seen = {false, false};

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
      model_received_ = model_received_ && seen[i];
  }

  void stateCb(const mavros_msgs::State::ConstPtr& msg, int i)
  {
    states_[i] = *msg;
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
        ROS_INFO("[formation2] %s OFFBOARD accepted attempt %d",
                 kModelNames[i].c_str(), attempt);
        streamFor(0.3);
        return true;
      }
      ROS_WARN("[formation2] %s OFFBOARD failed attempt %d", kModelNames[i].c_str(), attempt);
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
        ROS_INFO("[formation2] %s armed attempt %d", kModelNames[i].c_str(), attempt);
        streamFor(0.3);
        return true;
      }
      ROS_WARN("[formation2] %s arm failed attempt %d", kModelNames[i].c_str(), attempt);
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
    ros::Time start = ros::Time::now();
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
            requestOffboardOnce(i);
          if (states_[i].connected && !states_[i].armed)
            requestArmOnce(i);
        }
        last_retry = ros::Time::now();
      }

      rate.sleep();
    }
  }

  void reassertReadyForFlight()
  {
    if (stage_ == Stage::TAKEOFF)
      publishTakeoffTargets();
    else
      publishLocalHomePoseSetpoints();

    for (int i = 0; i < kUavCount; ++i)
    {
      if (!states_[i].connected) continue;

      if (states_[i].mode != "OFFBOARD")
      {
        if (requestOffboardOnce(i))
          ROS_INFO("[formation2] %s OFFBOARD re-request accepted", kModelNames[i].c_str());
        else
          ROS_WARN("[formation2] %s OFFBOARD re-request failed", kModelNames[i].c_str());
      }

      if (!states_[i].armed)
      {
        if (requestArmOnce(i))
          ROS_INFO("[formation2] %s arm re-request accepted", kModelNames[i].c_str());
        else
          ROS_WARN("[formation2] %s arm re-request failed", kModelNames[i].c_str());
      }
    }
  }

  bool allReady() const
  {
    if (!model_received_) return false;
    for (int i = 0; i < kUavCount; ++i)
    {
      if (!states_[i].connected || !states_[i].armed || states_[i].mode != "OFFBOARD")
        return false;
    }
    return true;
  }

  bool statesConnected() const
  {
    for (int i = 0; i < kUavCount; ++i)
      if (!states_[i].connected) return false;
    return true;
  }

  bool localPosesReceived() const
  {
    for (int i = 0; i < kUavCount; ++i)
      if (!local_pose_received_[i]) return false;
    return true;
  }

  void primeSetpoints(ros::Rate& rate)
  {
    ROS_INFO("[formation2] priming OFFBOARD setpoint stream for 2 seconds");
    ros::Time start = ros::Time::now();
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
    ros::Time start = ros::Time::now();
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
      pose_pubs_[i].publish(toPose(local_positions_[i].x, local_positions_[i].y,
                                   local_positions_[i].z, local_yaws_[i]));
    }
  }

  void publishLocalHomePoseSetpoints()
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(local_home_[i].x, local_home_[i].y,
                                   local_home_[i].z, home_yaws_[i]));
    }
  }

  void publishLocalPoseTargets(const std::array<Vec3, kUavCount>& targets)
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(targets[i].x, targets[i].y, targets[i].z, home_yaws_[i]));
    }
  }

  void publishVelocityToTargets(const std::array<Vec3, kUavCount>& targets)
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      Vec3 err = opSub(targets[i], positions_[i]);
      Vec3 v = opMul({err.x, err.y, 0.0}, 0.8);
      double speed = norm2d(v);
      if (speed > kMaxSpeed)
        v = opMul(v, kMaxSpeed / speed);

      double vz = clamp((kFlightZ - positions_[i].z) * 0.6, -0.4, 0.4);
      twist_pubs_[i].publish(toTwist(v.x, v.y, vz));
    }
  }

  void publishZeroVelocity()
  {
    for (int i = 0; i < kUavCount; ++i)
      twist_pubs_[i].publish(toTwist(0.0, 0.0, 0.0));
  }

  void enterStage(Stage next)
  {
    stage_ = next;
    stage_start_ = ros::Time::now();

    switch (stage_)
    {
      case Stage::TAKEOFF:
      {
        takeoff_start_z_ = local_positions_[0].z;
        for (int i = 0; i < kUavCount; ++i)
          takeoff_start_z_ = std::min(takeoff_start_z_, local_positions_[i].z);
        takeoff_target_z_ = takeoff_start_z_;
        takeoff_altitude_ready_since_ = ros::Time(0);
        ROS_INFO("[formation2] Stage TAKEOFF: climb from %.2f m to %.1f m",
                 takeoff_start_z_, kFlightZ);
        break;
      }
      case Stage::HOVER:
        ROS_INFO("[formation2] Stage HOVER: 3 seconds");
        break;
      case Stage::EXPAND_SHRINK:
        ROS_INFO("[formation2] Stage EXPAND_SHRINK: 2.0m radial and back");
        break;
      case Stage::ROTATE:
        captureRotateTargets();
        ROS_INFO("[formation2] Stage ROTATE: 180 deg around center");
        break;
      case Stage::LAND:
        landing_target_z_ = kFlightZ;
        ROS_INFO("[formation2] Landing");
        break;
      default:
        break;
    }
  }

  void runTakeoff()
  {
    double elapsed = stageElapsed();
    publishTakeoffTargets();
    bool altitude_ready = allAtAltitude(kFlightZ, kTakeoffAltitudeTolerance);

    if (altitude_ready)
    {
      if (takeoff_altitude_ready_since_.isZero())
        takeoff_altitude_ready_since_ = ros::Time::now();
    }
    else
    {
      takeoff_altitude_ready_since_ = ros::Time(0);
    }

    double stable_time = takeoff_altitude_ready_since_.isZero()
                           ? 0.0
                           : (ros::Time::now() - takeoff_altitude_ready_since_).toSec();

    ROS_INFO_THROTTLE(1.0,
                      "[formation2] TAKEOFF t=%.1f z=[%.2f %.2f] stable=%.1f",
                      elapsed, positions_[0].z, positions_[1].z, stable_time);

    if (!allReady() && (ros::Time::now() - last_ready_retry_).toSec() > 1.0)
    {
      reassertReadyForFlight();
      last_ready_retry_ = ros::Time::now();
    }

    if (altitude_ready && stable_time >= 1.0)
    {
      enterStage(Stage::HOVER);
      return;
    }

    if (elapsed > takeoff_timeout_)
    {
      if ((ros::Time::now() - last_ready_retry_).toSec() > 1.0)
      {
        reassertReadyForFlight();
        last_ready_retry_ = ros::Time::now();
      }
    }
  }

  void publishTakeoffTargets()
  {
    double elapsed = stageElapsed();
    takeoff_target_z_ = std::min(kFlightZ, takeoff_start_z_ + takeoff_climb_rate_ * elapsed);

    std::array<Vec3, kUavCount> targets = local_home_;
    for (int i = 0; i < kUavCount; ++i)
      targets[i].z = takeoff_target_z_;
    publishLocalPoseTargets(targets);
  }

  void runHover()
  {
    double elapsed = stageElapsed();
    publishLocalHomePoseSetpoints();
    ROS_INFO_THROTTLE(1.0, "[formation2] HOVER t=%.1f/3.0", elapsed);
    if (elapsed >= 3.0)
      enterStage(Stage::EXPAND_SHRINK);
  }

  void runExpandShrink()
  {
    double elapsed = stageElapsed();
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
    ROS_INFO_THROTTLE(1.0, "[formation2] EXPAND_SHRINK t=%.1f/10.0", elapsed);

    if (elapsed >= 10.0 && allAtTargets(home_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::ROTATE);
    }
  }

  void runRotate()
  {
    double elapsed = stageElapsed();
    if (elapsed < 5.0)
    {
      const double angle = kRotateAngle * clamp(elapsed / 5.0, 0.0, 1.0);
      publishVelocityToTargets(rotateTargets(angle));
    }
    else
    {
      const double angle = kRotateAngle * (1.0 - clamp((elapsed - 5.0) / 5.0, 0.0, 1.0));
      publishVelocityToTargets(rotateTargets(angle));
    }
    ROS_INFO_THROTTLE(1.0, "[formation2] ROTATE t=%.1f/10.0", elapsed);

    if (elapsed >= 10.0 && allAtTargets(rotate_start_positions_, 0.6) &&
        allAtAltitude(kFlightZ, kStageAltitudeTolerance))
    {
      publishZeroVelocity();
      enterStage(Stage::LAND);
    }
  }

  void runLand()
  {
    double elapsed = stageElapsed();
    double dt = 1.0 / kRateHz;
    landing_target_z_ = std::max(0.0, landing_target_z_ - 0.35 * dt);

    for (int i = 0; i < kUavCount; ++i)
    {
      pose_pubs_[i].publish(toPose(local_home_[i].x, local_home_[i].y,
                                   landing_target_z_, home_yaws_[i]));
    }

    ROS_INFO_THROTTLE(1.0,
                      "[formation2] LAND t=%.1f z=[%.2f %.2f]",
                      elapsed, positions_[0].z, positions_[1].z);

    if (allLanded() || elapsed > landing_timeout_)
    {
      if (elapsed > landing_timeout_)
        ROS_WARN("[formation2] landing timeout %.1f s", landing_timeout_);
      enterStage(Stage::DONE);
    }
  }

  bool allAtAltitude(double z, double tolerance) const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (std::fabs(positions_[i].z - z) > tolerance)
        return false;
    }
    return true;
  }

  bool allAtTargets(const std::array<Vec3, kUavCount>& targets, double tolerance) const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      Vec3 err = opSub(targets[i], positions_[i]);
      if (norm2d(err) > tolerance)
        return false;
    }
    return true;
  }

  bool allLanded() const
  {
    for (int i = 0; i < kUavCount; ++i)
    {
      if (positions_[i].z > 0.25)
        return false;
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
          ROS_INFO("[formation2] %s disarmed", kModelNames[i].c_str());
          break;
        }
        ros::Duration(0.3).sleep();
      }
    }
  }

  Vec3 formationCenter(const std::array<Vec3, kUavCount>& points) const
  {
    Vec3 c{0.0, 0.0, 0.0};
    for (int i = 0; i < kUavCount; ++i)
      c = opAdd(c, points[i]);
    return opMul(c, 1.0 / static_cast<double>(kUavCount));
  }

  void buildMotionTargets()
  {
    // Stage 3: 径向扩展 2.0m
    for (int i = 0; i < kUavCount; ++i)
    {
      Vec3 radial = normalize2d(opSub(home_[i], center_));
      expand_targets_[i] = opAdd(home_[i], opMul(radial, kExpandDistance));
      expand_targets_[i].z = kFlightZ;
    }
  }

  void captureRotateTargets()
  {
    // 记录进入 Stage 4 时的当前位置 (Gazebo 真值)
    for (int i = 0; i < kUavCount; ++i)
    {
      rotate_start_positions_[i] = positions_[i];
      rotate_start_positions_[i].z = kFlightZ;
    }

    // 以两机中心为原点，旋转 180 度后的目标位置
    rotate_center_ = formationCenter(rotate_start_positions_);
    for (int i = 0; i < kUavCount; ++i)
    {
      rotate_swap_targets_[i] = rotate2d(rotate_start_positions_[i], rotate_center_, kRotateAngle);
      rotate_swap_targets_[i].z = kFlightZ;
    }

    ROS_INFO("[formation2] rotate around (%.2f, %.2f): iris_0 (%.2f,%.2f)->(%.2f,%.2f), "
             "iris_1 (%.2f,%.2f)->(%.2f,%.2f)",
             rotate_center_.x, rotate_center_.y,
             rotate_start_positions_[0].x, rotate_start_positions_[0].y,
             rotate_swap_targets_[0].x, rotate_swap_targets_[0].y,
             rotate_start_positions_[1].x, rotate_start_positions_[1].y,
             rotate_swap_targets_[1].x, rotate_swap_targets_[1].y);
  }

  std::array<Vec3, kUavCount> rotateTargets(double angle_rad) const
  {
    std::array<Vec3, kUavCount> targets;
    for (int i = 0; i < kUavCount; ++i)
    {
      targets[i] = rotate2d(rotate_start_positions_[i], rotate_center_, angle_rad);
      targets[i].z = kFlightZ;
    }
    return targets;
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

  double stageElapsed() const
  {
    return (ros::Time::now() - stage_start_).toSec();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber gazebo_sub_;
  std::array<ros::Subscriber, kUavCount> state_subs_;
  std::array<ros::Subscriber, kUavCount> local_pose_subs_;
  std::array<ros::Publisher, kUavCount> pose_pubs_;
  std::array<ros::Publisher, kUavCount> twist_pubs_;
  std::array<ros::ServiceClient, kUavCount> arm_clients_;
  std::array<ros::ServiceClient, kUavCount> mode_clients_;

  std::array<Vec3, kUavCount> positions_;
  std::array<Vec3, kUavCount> local_positions_;
  std::array<Vec3, kUavCount> home_;
  std::array<Vec3, kUavCount> local_home_;
  std::array<Vec3, kUavCount> expand_targets_;
  std::array<Vec3, kUavCount> rotate_start_positions_;
  std::array<Vec3, kUavCount> rotate_swap_targets_;
  std::array<mavros_msgs::State, kUavCount> states_;
  std::array<double, kUavCount> local_yaws_;
  std::array<double, kUavCount> home_yaws_;
  std::array<bool, kUavCount> local_pose_received_;

  bool model_received_;
  Stage stage_;
  ros::Time stage_start_;
  ros::Time takeoff_altitude_ready_since_;
  ros::Time last_ready_retry_;
  Vec3 center_;
  Vec3 rotate_center_;
  double landing_target_z_;
  double takeoff_target_z_;
  double takeoff_start_z_;
  double takeoff_climb_rate_;
  double takeoff_timeout_;
  double landing_timeout_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_uavs_formation");
  TwoUavsFormation controller;
  controller.spin();
  return 0;
}
