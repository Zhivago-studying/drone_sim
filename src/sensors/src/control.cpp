/**
 * @file  control.cpp
 * @brief 四机编队控制器节点 (swarm_controller)
 *
 * =========================== 功能描述 ===========================
 * 集中式编队控制器，同时控制 4 架 iris 无人机完成全自主飞行流程。
 * 包含 6 阶段状态机：起飞 → 自旋 → 编队追逐(第1轮) → 径向扩展 →
 * 编队追逐(第2轮) → 降落。采用位置/速度混合控制，自动处理 PX4
 * OFFBOARD 模式切换、解锁、参数配置等底层操作。
 *
 * =========================== 数据流 ===========================
 * 订阅:
 *   /gazebo/model_states  (gazebo_msgs/ModelStates)
 *     - 获取所有无人机的 Gazebo 真值位置 (x, y, z)
 *     - 用于阶段切换条件判断和编队向量计算
 *
 * 发布:
 *   /iris_X/mavros/setpoint_position/local  (geometry_msgs/PoseStamped)
 *     - 起飞/降落阶段: 位置控制指令
 *   /iris_X/mavros/setpoint_velocity/cmd_vel  (geometry_msgs/TwistStamped)
 *     - 自旋/编队追逐/扩展阶段: 速度控制指令
 *
 * 服务调用:
 *   /iris_X/mavros/set_mode              (mavros_msgs/SetMode)
 *     - 切换 OFFBOARD 飞行模式
 *   /iris_X/mavros/cmd/arming            (mavros_msgs/CommandBool)
 *     - 解锁/上锁无人机
 *   /iris_X/mavros/param/set             (mavros_msgs/ParamSet)
 *     - 设置 PX4 参数 (COM_RCL_EXCEPT, COM_ARM_WO_GPS 等)
 *
 * =========================== 6 阶段状态机 ===========================
 *   Stage 1 – 起飞 (Take-off)        位置控制 z=3.0m, 全部>2.8m+5s 或超时20s
 *   Stage 2 – 原地自旋 (Yaw)         速度控制 angular.z=0.6, 30s / 3圈
 *   Stage 3 – 正方形追逐编队 第一轮  速度控制 逐机追踪, 25s → 悬停2s
 *   Stage 4 – 径向扩展               速度控制 背离中心 0.5m/s, 5s → 悬停2s
 *   Stage 5 – 正方形追逐编队 第二轮  同 Stage 3
 *   Stage 6 – 降落 (Land)            位置控制 z→0.3m, 落地后上锁退出
 *
 * =========================== 安全机制 ===========================
 *   - 2 秒 setpoint priming (OFFBOARD 前置条件)
 *   - 每架无人机逐架切换 OFFBOARD + 解锁 (5 次重试)
 *   - 8 项 PX4 参数预配置 (跳过 RC 检查、允许无 GPS 解锁等)
 *   - 编队控制 20Hz 实时闭环
 *
 * =========================== 启动方式 ===========================
 *   单节点集中式控制，无命名空间:
 *   <node pkg="sensors" type="control_node" name="swarm_controller"/>
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <cmath>
#include <string>
#include <array>

/* ================================================================== *
 *  数学 / 消息工具
 * ================================================================== */
struct Vec3 { double x, y, z; };

static Vec3  vadd(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static Vec3  vsub(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static Vec3  vmul(const Vec3& a, double s)       { return {a.x*s,  a.y*s,  a.z*s }; }

static double vlen(const Vec3& v)     { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }
static double vlenXY(const Vec3& v)   { return std::sqrt(v.x*v.x + v.y*v.y); }
static Vec3   vnorm(const Vec3& v)    {
  double n = vlen(v); if (n < 1e-8) return {0,0,0}; return vmul(v, 1.0/n);
}
static Vec3   vnormXY(const Vec3& v)  {
  double n = vlenXY(v); if (n < 1e-8) return {0,0,0}; return {v.x/n, v.y/n, 0};
}

static geometry_msgs::PoseStamped toPose(double x, double y, double z, double yaw) {
  geometry_msgs::PoseStamped p;
  p.header.stamp    = ros::Time::now();
  p.header.frame_id = "map";
  p.pose.position.x = x;  p.pose.position.y = y;  p.pose.position.z = z;
  p.pose.orientation.w = std::cos(yaw*0.5);
  p.pose.orientation.z = std::sin(yaw*0.5);
  return p;
}

static geometry_msgs::TwistStamped toTwist(double vx, double vy, double vz, double az) {
  geometry_msgs::TwistStamped t;
  t.header.stamp    = ros::Time::now();
  t.header.frame_id = "map";
  t.twist.linear.x  = vx;  t.twist.linear.y  = vy;  t.twist.linear.z  = vz;
  t.twist.angular.z = az;
  return t;
}

/* ================================================================== *
 *  常量
 * ================================================================== */
static constexpr int    UAV_N      = 4;
static constexpr double CTRL_HZ   = 20.0;
static constexpr double FLIGHT_Z  = 3.0;
static constexpr double MAX_SPEED = 1.0;    // m/s
static constexpr double YAW_RATE  = 0.6;    // rad/s
static constexpr double LAND_Z    = 0.3;    // 着陆判定高度

static const std::array<std::string, UAV_N> UAV_NAME =
  {"iris_0", "iris_1", "iris_2", "iris_3"};

/* ================================================================== *
 *  主控制器类
 * ================================================================== */
class SwarmController
{
public:
  SwarmController()
    : nh_("~"), stage_(STAGE_INIT), model_ok_(false), stage_t0_(ros::Time::now())
  {
    // 订阅 Gazebo 真实位置
    gz_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                            &SwarmController::gzCallback, this);

    // 初始化各无人机接口
    for (int i = 0; i < UAV_N; ++i) {
      std::string ns = "/" + UAV_NAME[i] + "/mavros";
      pose_pub_[i]  = nh_.advertise<geometry_msgs::PoseStamped>(
                        ns + "/setpoint_position/local", 10);
      vel_pub_[i]   = nh_.advertise<geometry_msgs::TwistStamped>(
                        ns + "/setpoint_velocity/cmd_vel", 10);
      arm_cli_[i]   = nh_.serviceClient<mavros_msgs::CommandBool>(
                        ns + "/cmd/arming");
      mode_cli_[i]  = nh_.serviceClient<mavros_msgs::SetMode>(
                        ns + "/set_mode");
      par_cli_[i]   = nh_.serviceClient<mavros_msgs::ParamSet>(
                        ns + "/param/set");
    }
  }

  /* ============================== 主循环 ============================== */
  void spin()
  {
    ros::Rate rate(CTRL_HZ);

    // ----- 等待 Gazebo 数据 -----
    ROS_INFO("[CTRL] Waiting for /gazebo/model_states ...");
    while (ros::ok() && !model_ok_) { ros::spinOnce(); rate.sleep(); }
    ROS_INFO("[CTRL] Model states received.");

    // 保存起飞前水平位置
    for (int i = 0; i < UAV_N; ++i) home_[i] = pos_[i];

    // ----- 设置 PX4 安全参数 -----
    ROS_INFO("[CTRL] Configuring PX4 params ...");
    for (int i = 0; i < UAV_N; ++i) configureParams(i);

    // ----- Priming: 持续发布 setpoint 2 秒 (OFFBOARD 前置条件) -----
    ROS_INFO("[CTRL] Priming setpoints (2 s) ...");
    ros::Time t0 = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t0).toSec() < 2.0) {
      ros::spinOnce();
      for (int i = 0; i < UAV_N; ++i)
        pose_pub_[i].publish(toPose(pos_[i].x, pos_[i].y, pos_[i].z, 0));
      rate.sleep();
    }

    // ----- OFFBOARD + ARM （逐架，setpoint 流持续） -----
    ROS_INFO("[CTRL] OFFBOARD + ARM (one-by-one) ...");
    for (int i = 0; i < UAV_N; ++i) {
      // 小段 priming 确保本机流稳定
      t0 = ros::Time::now();
      while (ros::ok() && (ros::Time::now() - t0).toSec() < 0.3) {
        ros::spinOnce();
        pose_pub_[i].publish(toPose(pos_[i].x, pos_[i].y, pos_[i].z, 0));
        rate.sleep();
      }
      setOffboard(i);
      t0 = ros::Time::now();
      while (ros::ok() && (ros::Time::now() - t0).toSec() < 0.5) {
        ros::spinOnce();
        pose_pub_[i].publish(toPose(pos_[i].x, pos_[i].y, pos_[i].z, 0));
        rate.sleep();
      }
      doArm(i);
      ros::Duration(0.3).sleep();
    }

    // ----- 进入 Stage 1 -----
    enterStage(1);

    /* ======================== 状态机主循环 ======================== */
    while (ros::ok())
    {
      ros::spinOnce();
      double t = (ros::Time::now() - stage_t0_).toSec();

      switch (stage_)
      {
      /* ========== Stage 1 – 起飞 ========== */
      case 1:
      {
        ROS_DEBUG_THROTTLE(2.0,
          "[CTRL] STAGE1-TAKEOFF  t=%.1f  z=[%.1f,%.1f,%.1f,%.1f]",
          t, pos_[0].z, pos_[1].z, pos_[2].z, pos_[3].z);

        // 位置控制：保持起飞前 x,y，目标高度 3.0m
        for (int i = 0; i < UAV_N; ++i)
          pose_pub_[i].publish(toPose(home_[i].x, home_[i].y, FLIGHT_Z, 0));

        // 判断：全部高度 > 2.8m 且稳定 5s，或超时 20s
        if (t > 5.0) {
          bool all_up = true;
          for (int i = 0; i < UAV_N; ++i)
            if (pos_[i].z < 2.8) { all_up = false; break; }
          if (all_up || t > 20.0) {
            ROS_INFO("[CTRL] STAGE1 done  (%s)", all_up ? "OK" : "timeout");
            enterStage(2);
          }
        }
        break;
      }

      /* ========== Stage 2 – 原地自旋 3圈 / 30s ========== */
      case 2:
      {
        ROS_DEBUG_THROTTLE(2.0,
          "[CTRL] STAGE2-YAW  t=%.1f  rev=%.1f", t, t * YAW_RATE / (2*M_PI));

        // 速度控制：线速度 0，偏航角速度 0.6 rad/s
        for (int i = 0; i < UAV_N; ++i)
          vel_pub_[i].publish(toTwist(0, 0, 0, YAW_RATE));

        if (t > 30.0) {
          // 停止旋转
          for (int i = 0; i < UAV_N; ++i)
            vel_pub_[i].publish(toTwist(0, 0, 0, 0));
          ROS_INFO("[CTRL] STAGE2 done");
          enterStage(3);
        }
        break;
      }

      /* ========== Stage 3 – 正方形追逐编队 第一轮 ========== */
      case 3:
      {
        ROS_DEBUG_THROTTLE(2.0,
          "[CTRL] STAGE3-FORM1  t=%.1f  chase", t);

        // 25s 追逐 + 2s 悬停
        if (t < 25.0) {
          // 追逐：每机朝下一机方向飞 1.0 m/s
          for (int i = 0; i < UAV_N; ++i) {
            int next = (i + 1) % UAV_N;
            Vec3 dir = vsub(pos_[next], pos_[i]);
            dir = vnormXY(dir);
            vel_pub_[i].publish(toTwist(dir.x * MAX_SPEED, dir.y * MAX_SPEED, 0, 0));
          }
        } else {
          // 悬停 2s
          for (int i = 0; i < UAV_N; ++i)
            vel_pub_[i].publish(toTwist(0, 0, 0, 0));

          if (t > 27.0) {
            ROS_INFO("[CTRL] STAGE3 done");
            enterStage(4);
          }
        }
        break;
      }

      /* ========== Stage 4 – 径向扩展 ========== */
      case 4:
      {
        // 计算四机中心
        Vec3 ctr = {0, 0, 0};
        for (int i = 0; i < UAV_N; ++i) ctr = vadd(ctr, pos_[i]);
        ctr = vmul(ctr, 1.0 / UAV_N);

        ROS_DEBUG_THROTTLE(2.0,
          "[CTRL] STAGE4-EXPAND  t=%.1f  ctr=(%.1f,%.1f)", t, ctr.x, ctr.y);

        // 5s 扩展 + 2s 悬停
        if (t < 5.0) {
          // 背离中心方向，速度 0.5 m/s
          for (int i = 0; i < UAV_N; ++i) {
            Vec3 dir = vsub(pos_[i], ctr);
            dir = vnormXY(dir);
            vel_pub_[i].publish(toTwist(dir.x * 0.5, dir.y * 0.5, 0, 0));
          }
        } else {
          for (int i = 0; i < UAV_N; ++i)
            vel_pub_[i].publish(toTwist(0, 0, 0, 0));

          if (t > 7.0) {
            ROS_INFO("[CTRL] STAGE4 done");
            enterStage(5);
          }
        }
        break;
      }

      /* ========== Stage 5 – 正方形追逐编队 第二轮 ========== */
      case 5:
      {
        ROS_DEBUG_THROTTLE(2.0,
          "[CTRL] STAGE5-FORM2  t=%.1f  chase", t);

        if (t < 25.0) {
          for (int i = 0; i < UAV_N; ++i) {
            int next = (i + 1) % UAV_N;
            Vec3 dir = vsub(pos_[next], pos_[i]);
            dir = vnormXY(dir);
            vel_pub_[i].publish(toTwist(dir.x * MAX_SPEED, dir.y * MAX_SPEED, 0, 0));
          }
        } else {
          for (int i = 0; i < UAV_N; ++i)
            vel_pub_[i].publish(toTwist(0, 0, 0, 0));

          if (t > 27.0) {
            ROS_INFO("[CTRL] STAGE5 done");
            enterStage(6);
          }
        }
        break;
      }

      /* ========== Stage 6 – 降落 ========== */
      case 6:
      {
        ROS_DEBUG_THROTTLE(2.0,
          "[CTRL] STAGE6-LAND  t=%.1f  z=[%.1f,%.1f,%.1f,%.1f]",
          t, pos_[0].z, pos_[1].z, pos_[2].z, pos_[3].z);

        // 位置控制：逐步降低高度（每周期降 0.15m/s * dt）
        double dz = 0.15 / CTRL_HZ;  // 0.15 m/s 下降速率
        for (int i = 0; i < UAV_N; ++i) {
          double target_z = std::max(0.0, pos_[i].z - dz);
          pose_pub_[i].publish(toPose(pos_[i].x, pos_[i].y, target_z, 0));
        }

        // 判断全部落地
        if (t > 2.0) {
          bool all_landed = true;
          for (int i = 0; i < UAV_N; ++i)
            if (pos_[i].z > LAND_Z) { all_landed = false; break; }

          if (all_landed || t > 30.0) {
            ROS_INFO("[CTRL] STAGE6 done  (%s) – disarming",
                     all_landed ? "landed" : "timeout");
            // 上锁所有无人机
            for (int i = 0; i < UAV_N; ++i) disarm(i);
            ros::Duration(1.0).sleep();
            ROS_INFO("[CTRL] Mission complete.");
            ros::shutdown();
            return;
          }
        }
        break;
      }
      } // switch

      rate.sleep();
    } // while
  }

private:
  /* ============================== 回调 ============================== */
  void gzCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    int found = 0;
    for (int i = 0; i < UAV_N; ++i) {
      for (size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] == UAV_NAME[i]) {
          pos_[i] = {msg->pose[j].position.x,
                     msg->pose[j].position.y,
                     msg->pose[j].position.z};
          ++found;
          break;
        }
      }
    }
    if (found >= UAV_N) model_ok_ = true;
  }

  /* ============================== 阶段切换 ============================== */
  void enterStage(int s)
  {
    stage_    = s;
    stage_t0_ = ros::Time::now();
    ROS_INFO("[CTRL] >>> entering Stage %d <<<", s);
  }

  /* ============================== 参数设置 ============================== */
  void configureParams(int i)
  {
    auto setInt = [&](const std::string& name, int val) {
      mavros_msgs::ParamSet srv;
      srv.request.param_id      = name;
      srv.request.value.integer = val;
      par_cli_[i].call(srv);
      ros::Duration(0.08).sleep();
    };
    auto setReal = [&](const std::string& name, double val) {
      mavros_msgs::ParamSet srv;
      srv.request.param_id   = name;
      srv.request.value.real = val;
      par_cli_[i].call(srv);
      ros::Duration(0.08).sleep();
    };

    setReal("COM_OF_LOSS_T",   5.0);
    setInt ("COM_OBL_ACT",     1);
    setInt ("COM_OBL_RC_ACT",  0);
    setInt ("COM_RCL_EXCEPT",  4);    // ★ 跳过 RC 遥控器检查
    setInt ("COM_RC_IN_MODE",  1);
    setInt ("COM_RC_OVERRIDE", 1);
    setInt ("COM_ARM_WO_GPS",  1);    // 允许无 GPS 解锁
    setInt ("NAV_RCL_ACT",     0);
    ROS_DEBUG("  %s: params configured", UAV_NAME[i].c_str());
  }

  /* ============================== OFFBOARD / ARM ============================== */
  void setOffboard(int i)
  {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = "OFFBOARD";
    for (int n = 0; n < 5 && ros::ok(); ++n) {
      if (mode_cli_[i].call(srv) && srv.response.mode_sent) {
        ROS_DEBUG("  %s: OFFBOARD OK (attempt %d)", UAV_NAME[i].c_str(), n+1);
        return;
      }
      ros::Duration(0.3).sleep();
    }
    ROS_WARN("  %s: OFFBOARD FAILED", UAV_NAME[i].c_str());
  }

  void doArm(int i)
  {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    for (int n = 0; n < 5 && ros::ok(); ++n) {
      if (arm_cli_[i].call(srv) && srv.response.success) {
        ROS_DEBUG("  %s: ARM OK (attempt %d)", UAV_NAME[i].c_str(), n+1);
        return;
      }
      ros::Duration(0.3).sleep();
    }
    ROS_WARN("  %s: ARM FAILED", UAV_NAME[i].c_str());
  }

  void disarm(int i)
  {
    mavros_msgs::CommandBool srv;
    srv.request.value = false;
    for (int n = 0; n < 5 && ros::ok(); ++n) {
      if (arm_cli_[i].call(srv) && srv.response.success) {
        ROS_DEBUG("  %s: DISARM OK", UAV_NAME[i].c_str());
        return;
      }
      ros::Duration(0.3).sleep();
    }
    ROS_WARN("  %s: DISARM FAILED", UAV_NAME[i].c_str());
  }

  /* ============================== 成员变量 ============================== */
  ros::NodeHandle nh_;
  ros::Subscriber gz_sub_;

  ros::Publisher  pose_pub_[UAV_N];
  ros::Publisher  vel_pub_[UAV_N];
  ros::ServiceClient arm_cli_[UAV_N];
  ros::ServiceClient mode_cli_[UAV_N];
  ros::ServiceClient par_cli_[UAV_N];

  Vec3 pos_[UAV_N];      // 当前真实位置（来自 Gazebo）
  Vec3 home_[UAV_N];     // 起飞前水平位置

  int  stage_;
  ros::Time stage_t0_;
  bool model_ok_;

  static constexpr int STAGE_INIT = 0;
};

/* ================================================================== */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_controller");
  SwarmController ctrl;
  ctrl.spin();
  return 0;
}
