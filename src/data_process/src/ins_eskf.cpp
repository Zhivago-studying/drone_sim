/**
 * INS ESKF (Error-State Extended Kalman Filter)
 *
 * 仅使用 IMU + PX4FLOW 光流 + ToF 高度完成状态估计.
 *
 * 状态定义:
 *   名义状态 (16维): p(3), v(3), q(4), ba(3), bg(3)
 *   误差状态 (15维): dp(3), dv(3), dtheta(3), dba(3), dbg(3)
 *
 * 预测:  IMU 驱动, 基于加速度/角速度递推名义状态 + 传播误差协方差
 * 更新:  光流速度 (body系 vx/vy) + ToF 高度 (pz)
 *
 * 坐标系:
 *   世界: ENU (x=East, y=North, z=Up), 重力 g=(0,0,-9.81)
 *   机体: FLU (x=Forward, y=Left, z=Up) — 与 MAVROS odom 一致
 *
 *   MAVROS odom 姿态 = body_FLU → ENU  (ROS 标准, 已转换)
 *   MAVROS sensor_msgs/Imu 默认已经是 ROS base_link/FLU 约定.
 *   若接入的是 PX4 原生 FRD 数据，可通过 imu_frame=frd 或 imu_frame=auto 转换.
 *
 * 光流恢复 (MAVROS OpticalFlowRad, body FLU):
 *   vx_flu = h*(flow_x - w_x_flu)
 *   vy_flu = h*(flow_y + w_y_flu)
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <sys/stat.h>

// =============================================================================
//  辅助函数
// =============================================================================

/** 叉乘矩阵: Skew(v) * x = v × x */
inline Eigen::Matrix3d Skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d M;
    M <<    0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return M;
}

/** SO(3) 指数映射: 旋转向量 theta -> 单位四元数 */
inline Eigen::Quaterniond Exp(const Eigen::Vector3d &theta)
{
    double angle = theta.norm();
    if (angle < 1e-12)
        return Eigen::Quaterniond::Identity();
    Eigen::Vector3d axis = theta / angle;
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

/** 从四元数提取 RPY (Euler ZYX), 返回角度 (deg) */
inline Eigen::Vector3d quatToRPY(const Eigen::Quaterniond &q)
{
    // Eigen quaternion: (w,x,y,z), ZYX convention
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler * 180.0 / M_PI;  // rad -> deg
}

// =============================================================================
//  InsEkf 类
// =============================================================================

class InsEkf
{
public:
    InsEkf() : nh_("~")
    {
        // 获取命名空间 (eg. /iris_0 → "/iris_0")
        ns_ = ros::this_node::getNamespace();

        // 调试/坐标系开关
        nh_.param("disable_attitude_update", disable_att_, true);
        nh_.param<std::string>("imu_frame", imu_frame_param_, "auto");  // auto|flu|frd
        nh_.param("min_flow_quality", min_flow_quality_, 100);
        nh_.param("max_innovation_sigma", max_innovation_sigma_, 6.0);
        nh_.param("publish_pos_cov_scale", publish_pos_cov_scale_, 15.0);
        nh_.param("publish_vel_cov_scale", publish_vel_cov_scale_, 3.0);
        nh_.param("flow_relative_noise_std", flow_relative_noise_std_, 0.1);
        nh_.param("flow_base_noise_std", flow_base_noise_std_, 0.02);
        nh_.param("tof_noise_std", tof_noise_std_, 0.05);
        nh_.param("tof_min_range", tof_min_range_, 0.20);
        nh_.param("initial_attitude_std_deg", initial_attitude_std_deg_, 3.0);
        // Keep the legacy parameter as a fallback for older launch files.
        nh_.param("attitude_update_min_speed",
                  attitude_update_min_vertical_speed_, 0.20);
        nh_.param("attitude_update_min_vertical_speed",
                  attitude_update_min_vertical_speed_,
                  attitude_update_min_vertical_speed_);
        nh_.param("attitude_update_max_innovation",
                  attitude_update_max_innovation_, 0.15);
        nh_.param("attitude_update_max_correction_deg",
                  attitude_update_max_correction_deg_, 0.15);
        nh_.param("initial_gyro_bias_std", initial_gyro_bias_std_, 0.001);
        nh_.param("max_velocity_innovation", max_velocity_innovation_, 0.75);
        nh_.param("velocity_recovery_reject_count",
                  velocity_recovery_reject_count_, 5);
        nh_.param("max_recovery_velocity_correction",
                  max_recovery_velocity_correction_, 0.10);
        nh_.param("max_position_correction", max_position_correction_, 0.25);
        nh_.param("max_velocity_correction", max_velocity_correction_, 0.40);
        nh_.param("max_accel_bias_correction",
                  max_accel_bias_correction_, 0.02);
        nh_.param("max_gyro_bias_correction",
                  max_gyro_bias_correction_, 0.0002);
        nh_.param("max_accel_bias_norm", max_accel_bias_norm_, 0.25);
        nh_.param("max_gyro_bias_norm", max_gyro_bias_norm_, 0.003);
        nh_.param("disable_yaw_bias_update", disable_yaw_bias_update_, true);
        nh_.param("measurement_noise_seed", measurement_noise_seed_, 1);
        nh_.param("flow_csv_dir", flow_csv_dir_, std::string(""));
        if (min_flow_quality_ < 0 ||
            max_innovation_sigma_ <= 0.0 ||
            publish_pos_cov_scale_ <= 0.0 ||
            publish_vel_cov_scale_ <= 0.0 ||
            flow_relative_noise_std_ < 0.0 ||
            flow_base_noise_std_ < 0.0 ||
            tof_noise_std_ < 0.0 ||
            tof_min_range_ < 0.0 ||
            initial_attitude_std_deg_ <= 0.0 ||
            attitude_update_min_vertical_speed_ < 0.0 ||
            attitude_update_max_innovation_ <= 0.0 ||
            attitude_update_max_correction_deg_ <= 0.0 ||
            initial_gyro_bias_std_ <= 0.0 ||
            max_velocity_innovation_ <= 0.0 ||
            velocity_recovery_reject_count_ <= 0 ||
            max_recovery_velocity_correction_ <= 0.0 ||
            max_position_correction_ <= 0.0 ||
            max_velocity_correction_ <= 0.0 ||
            max_accel_bias_correction_ <= 0.0 ||
            max_gyro_bias_correction_ <= 0.0 ||
            max_accel_bias_norm_ <= 0.0 ||
            max_gyro_bias_norm_ <= 0.0)
        {
            ROS_FATAL("[INS ESKF] invalid measurement/update parameter: "
                      "min_flow_quality=%d max_innovation_sigma=%.3f "
                      "publish_cov_scale=(%.3f,%.3f) "
                      "flow_relative_noise_std=%.3f flow_base_noise_std=%.3f "
                      "tof_noise_std=%.3f tof_min_range=%.3f "
                      "initial_attitude_std_deg=%.3f "
                      "attitude_update_min_vertical_speed=%.3f "
                      "attitude_update_max_innovation=%.3f "
                      "attitude_update_max_correction_deg=%.3f "
                      "initial_gyro_bias_std=%.4f "
                      "max_velocity_innovation=%.3f "
                      "velocity_recovery_reject_count=%d",
                      min_flow_quality_, max_innovation_sigma_,
                      publish_pos_cov_scale_, publish_vel_cov_scale_,
                      flow_relative_noise_std_, flow_base_noise_std_,
                      tof_noise_std_, tof_min_range_,
                      initial_attitude_std_deg_,
                      attitude_update_min_vertical_speed_,
                      attitude_update_max_innovation_,
                      attitude_update_max_correction_deg_,
                      initial_gyro_bias_std_,
                      max_velocity_innovation_,
                      velocity_recovery_reject_count_);
            throw std::runtime_error("invalid ESKF parameter");
        }
        if (measurement_noise_seed_ >= 0)
        {
            // Stable per-UAV seed makes A/B experiments reproducible while
            // preserving independent artificial noise between vehicles.
            std::uint32_t namespace_hash = 2166136261u;
            for (char c : ns_)
            {
                namespace_hash ^= static_cast<std::uint8_t>(c);
                namespace_hash *= 16777619u;
            }
            flow_noise_rng_.seed(
                static_cast<std::uint32_t>(measurement_noise_seed_) ^
                namespace_hash);
        }
        else
        {
            std::random_device random_device;
            flow_noise_rng_.seed(random_device());
        }

        // 光流诊断 CSV
        initFlowCsv();

        // --- 连续时间 IMU 过程噪声，与 Gazebo IMU 插件参数一致 ---
        nh_.param("sigma_acc",  sigma_acc_,  6.867e-4);
        nh_.param("sigma_gyro", sigma_gyro_, 4.8869e-5);
        nh_.param("sigma_ba",   sigma_ba_,   0.006);
        nh_.param("sigma_bg",   sigma_bg_,   3.8785e-5);
        nh_.param("tau_ba",     tau_ba_,     300.0);
        nh_.param("tau_bg",     tau_bg_,     1000.0);
        if (sigma_acc_ < 0.0 || sigma_gyro_ < 0.0 ||
            sigma_ba_ < 0.0 || sigma_bg_ < 0.0 ||
            tau_ba_ <= 0.0 || tau_bg_ <= 0.0)
        {
            ROS_FATAL("[INS ESKF] invalid IMU process parameter: "
                      "sigma_acc=%.3g sigma_gyro=%.3g sigma_ba=%.3g "
                      "sigma_bg=%.3g tau_ba=%.3f tau_bg=%.3f",
                      sigma_acc_, sigma_gyro_, sigma_ba_, sigma_bg_,
                      tau_ba_, tau_bg_);
            throw std::runtime_error("invalid IMU process parameter");
        }

        // --- 观测噪声 (3×3) ---
        R_meas_.setZero();
        R_meas_(0,0) = flow_base_noise_std_ * flow_base_noise_std_;
        R_meas_(1,1) = flow_base_noise_std_ * flow_base_noise_std_;
        R_meas_(2,2) = tof_noise_std_ * tof_noise_std_; // ToF 高度

        // --- 重力 (ENU) ---
        g_ << 0.0, 0.0, -9.81;

        // --- 初始化状态 ---
        p_.setZero();
        v_.setZero();
        q_.setIdentity();
        ba_.setZero();
        bg_.setZero();

        // 初始协方差。姿态来自 PX4 odom，不应再声明 1 rad 的不确定度；
        // 过大的 P_theta 会使低速光流噪声被解释为几十度姿态修正。
        P_ = Eigen::Matrix<double, 15, 15>::Identity();
        P_.block<3,3>(0, 0) *= 4.0;     // δp
        P_.block<3,3>(3, 3) *= 2.0;     // δv
        const double initial_attitude_std =
            initial_attitude_std_deg_ * M_PI / 180.0;
        P_.block<3,3>(6, 6) *=
            initial_attitude_std * initial_attitude_std; // δθ
        P_.block<3,3>(9, 9) *= 0.01;    // δba
        P_.block<3,3>(12,12) *=
            initial_gyro_bias_std_ * initial_gyro_bias_std_; // δbg

        ROS_INFO("[INS ESKF] ns=%s sigma_acc=%.2e sigma_gyro=%.2e "
                 "sigma_ba=%.2e tau_ba=%.1fs sigma_bg=%.2e tau_bg=%.1fs "
                 "flow_relative_noise_std=%.2f flow_base_noise_std=%.3fm/s "
                 "tof_noise_std=%.3fm tof_min_range=%.2fm "
                 "publish_cov_scale=(%.1f,%.1f) "
                 "disable_att_update=%d disable_yaw_bias_update=%d "
                 "initial_attitude_std=%.2fdeg initial_bg_std=%.4frad/s "
                 "min_att_vertical_speed=%.2fm/s "
                 "max_att_innovation=%.2fm/s max_att_correction=%.2fdeg "
                 "max_vel_innovation=%.2fm/s recovery_rejects=%d "
                 "noise_seed=%d imu_frame=%s",
                 ns_.c_str(), sigma_acc_, sigma_gyro_,
                 sigma_ba_, tau_ba_, sigma_bg_, tau_bg_,
                 flow_relative_noise_std_, flow_base_noise_std_,
                 tof_noise_std_, tof_min_range_,
                 publish_pos_cov_scale_, publish_vel_cov_scale_,
                 disable_att_, disable_yaw_bias_update_,
                 initial_attitude_std_deg_, initial_gyro_bias_std_,
                 attitude_update_min_vertical_speed_,
                 attitude_update_max_innovation_,
                 attitude_update_max_correction_deg_,
                 max_velocity_innovation_, velocity_recovery_reject_count_,
                 measurement_noise_seed_,
                 imu_frame_param_.c_str());

        // --- 订阅 ---
        odom_init_sub_ = nh_.subscribe(ns_ + "/mavros/local_position/odom", 1,
                                       &InsEkf::odomInitCb, this);
        imu_sub_  = nh_.subscribe(ns_ + "/mavros/imu/data_raw", 100,
                                  &InsEkf::imuCallback, this);
        flow_sub_ = nh_.subscribe(ns_ + "/mavros/px4flow/raw/optical_flow_rad", 10,
                                  &InsEkf::flowCallback, this);

        // --- 发布 ---
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(ns_ + "/ins_estimate", 10);
    }

    ~InsEkf()
    {
        if (flow_csv_.is_open())
            flow_csv_.close();
    }

private:
    struct VelocityUpdateDiagnostics
    {
        bool accepted = false;
        bool attitude_enabled = false;
        bool recovery_update = false;
        double nis = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector3d dtheta{0.0, 0.0, 0.0};
    };

    // =====================================================================
    //  Odom 初始化回调 — 获取初始姿态
    //  MAVROS 已输出 FLU→ENU 四元数, 直接赋值
    // =====================================================================

    void odomInitCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (!odom_inited_)
        {
            p_.x() = msg->pose.pose.position.x;
            p_.y() = msg->pose.pose.position.y;
            p_.z() = msg->pose.pose.position.z;
            p0_ = p_;
            v_.x() = msg->twist.twist.linear.x;
            v_.y() = msg->twist.twist.linear.y;
            v_.z() = msg->twist.twist.linear.z;

            q_.w() = msg->pose.pose.orientation.w;
            q_.x() = msg->pose.pose.orientation.x;
            q_.y() = msg->pose.pose.orientation.y;
            q_.z() = msg->pose.pose.orientation.z;
            odom_inited_ = true;

            Eigen::Vector3d rpy = quatToRPY(q_);
            ROS_INFO("[INS ESKF] state initialized from odom: p0=(%.2f, %.2f, %.2f) v=(%.2f, %.2f, %.2f) "
                     "q=(%.3f, %.3f, %.3f, %.3f) RPY(deg)=(%.1f, %.1f, %.1f)",
                     p0_.x(), p0_.y(), p0_.z(), v_.x(), v_.y(), v_.z(),
                     q_.w(), q_.x(), q_.y(), q_.z(), rpy.x(), rpy.y(), rpy.z());
        }

        // 持续缓存 GT 用于光流诊断 CSV
        gt_stamp_ = msg->header.stamp;
        gt_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
        gt_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y,
                                  msg->twist.twist.linear.z);
        gt_quat_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
        has_gt_ = true;
    }

    bool resolveImuFrame(const Eigen::Vector3d &acc_msg,
                         const Eigen::Vector3d &gyro_msg)
    {
        std::string frame = imu_frame_param_;
        std::transform(frame.begin(), frame.end(), frame.begin(), ::tolower);

        if (frame == "flu" || frame == "ros" || frame == "base_link")
        {
            imu_is_frd_ = false;
            imu_frame_resolved_ = true;
        }
        else if (frame == "frd" || frame == "px4" || frame == "aircraft")
        {
            imu_is_frd_ = true;
            imu_frame_resolved_ = true;
        }
        else
        {
            imu_acc_sum_ += acc_msg;
            imu_gyro_sum_ += gyro_msg;
            imu_acc_samples_++;
            if (imu_acc_samples_ < 20)
                return false;

            Eigen::Vector3d mean = imu_acc_sum_ / static_cast<double>(imu_acc_samples_);
            Eigen::Vector3d mean_gyro = imu_gyro_sum_ / static_cast<double>(imu_acc_samples_);
            imu_is_frd_ = mean.z() < 0.0;
            if (imu_is_frd_)
            {
                mean.y() = -mean.y();
                mean.z() = -mean.z();
                mean_gyro.y() = -mean_gyro.y();
                mean_gyro.z() = -mean_gyro.z();
            }

            ba_ = mean - q_.toRotationMatrix().transpose() * (-g_);
            // The startup gyro mean is close to MAVROS/SITL timestamp noise scale.
            // Keep bg at zero unless a longer stationary calibration is added.
            bg_.setZero();
            imu_frame_resolved_ = true;
            ROS_INFO("[INS ESKF] auto IMU frame: mean_acc_flu=(%.3f, %.3f, %.3f), "
                     "mean_gyro_flu=(%.4f, %.4f, %.4f), using %s, "
                     "init_ba=(%.3f, %.3f, %.3f), init_bg=(%.4f, %.4f, %.4f)",
                     mean.x(), mean.y(), mean.z(),
                     mean_gyro.x(), mean_gyro.y(), mean_gyro.z(),
                     imu_is_frd_ ? "FRD->FLU" : "FLU",
                     ba_.x(), ba_.y(), ba_.z(),
                     bg_.x(), bg_.y(), bg_.z());
        }

        if (imu_frame_resolved_ && frame != "auto")
        {
            ROS_INFO("[INS ESKF] configured IMU frame: using %s",
                     imu_is_frd_ ? "FRD->FLU" : "FLU");
        }
        return true;
    }

    // =====================================================================
    //  IMU 回调 — 执行预测步骤
    //
    //  关键: EKF 内部统一使用 FLU. MAVROS sensor_msgs/Imu 通常已经是 FLU;
    //        若接入 PX4 原生 FRD, resolveImuFrame() 会转换到 FLU.
    // =====================================================================

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        if (!odom_inited_)
            return;

        if (!has_imu_init_)
        {
            last_imu_time_ = msg->header.stamp;
            has_imu_init_ = true;
            ROS_DEBUG("[INS ESKF] IMU initialized at t=%.3f", last_imu_time_.toSec());
            return;
        }

        double dt = (msg->header.stamp - last_imu_time_).toSec();
        if (dt <= 0.0 || dt > 1.0)
        {
            last_imu_time_ = msg->header.stamp;
            return;
        }
        last_imu_time_ = msg->header.stamp;

        Eigen::Vector3d acc_msg(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
        Eigen::Vector3d gyro_msg(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);

        if (!imu_frame_resolved_)
        {
            if (!resolveImuFrame(acc_msg, gyro_msg))
                return;
        }

        Eigen::Vector3d acc = acc_msg;
        Eigen::Vector3d gyro = gyro_msg;
        if (imu_is_frd_)
        {
            acc.y()  = -acc.y();
            acc.z()  = -acc.z();
            gyro.y() = -gyro.y();
            gyro.z() = -gyro.z();
        }

        // 存储陀螺 (FLU, 调试/扩展用)
        latest_gyro_ = gyro;

        // SDF IMU plugin: deterministic part of first-order Gauss-Markov bias.
        ba_ *= std::exp(-dt / tau_ba_);
        bg_ *= std::exp(-dt / tau_bg_);

        // --- 名义状态预测 ---
        predictNominal(acc, gyro, dt);

        // --- 误差协方差传播 ---
        predictCovariance(acc, gyro, dt);

        // --- 发布当前估计 ---
        publishEstimate(msg->header.stamp);
    }

    // =====================================================================
    //  光流 + ToF 回调 — 执行 EKF 更新
    // =====================================================================

    void flowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr &msg)
    {
        if (msg->quality < min_flow_quality_)
            return;
        if (!std::isfinite(msg->distance) || msg->distance < 0.05 || msg->distance > 20.0)
            return;

        // PX4Flow/Gazebo 在量程下限附近输出硬饱和值。饱和段不能继续
        // 叠加零均值噪声，否则截断后的测距会产生正偏，并被高度更新长期积累。
        const bool tof_saturated =
            msg->distance < tof_min_range_ + 0.03;
        const double tof_noise = tof_saturated
                                     ? 0.0
                                     : tof_noise_std_ *
                                           standard_normal_(flow_noise_rng_);
        // 该模型的 ToF 安装在机体下方，落地时真实射线距离接近 0m，
        // 但 MAVROS 会发布约 0.20m 的硬钳位值。因此饱和值对应的有效
        // 相对距离是 0m，而不是 0.20m。
        const double distance_meas = tof_saturated
                                         ? 0.0
                                         : msg->distance + tof_noise;
        if (!std::isfinite(distance_meas) ||
            (!tof_saturated && distance_meas < 0.05) ||
            distance_meas > 20.0)
        {
            ROS_WARN_THROTTLE(10.0,
                              "[INS ESKF] noisy ToF distance invalid: raw=%.3f "
                              "noise=%.3f meas=%.3f",
                              msg->distance, tof_noise, distance_meas);
            return;
        }

        if (!height_initialized_)
        {
            height_ref_z_ = p_.z();
            if (tof_saturated)
            {
                // 饱和值已经转换成有效距离 0m；后续每个饱和样本也会
                // 保持为 0m，不会再把原始 0.20m 注入高度状态。
                height0_ = 0.0;
                ROS_INFO("[INS ESKF] ToF saturated at %.3f m (min %.2f m), "
                         "map saturated range to 0m; ref_z=%.3f",
                         msg->distance, tof_min_range_, height_ref_z_);
            }
            else
            {
                // Keep the artificial one-shot ToF noise out of the persistent datum.
                height0_ = msg->distance;
                ROS_DEBUG("[INS ESKF] height0 = %.3f m "
                          "(initial injected noise %.3f ignored), "
                          "height_ref_z = %.3f m",
                          height0_, tof_noise, height_ref_z_);
            }
            height_initialized_ = true;
            return;
        }

        if (!has_imu_init_)
            return;

        double dt_flow = msg->integration_time_us * 1e-6;
        if (dt_flow < 1e-6)
            return;

        // 反积分得到光流角速率和同一积分窗口内的 gyro 角速率
        double flow_x = msg->integrated_x / dt_flow;
        double flow_y = msg->integrated_y / dt_flow;
        double gyro_x = msg->integrated_xgyro / dt_flow;
        double gyro_y = msg->integrated_ygyro / dt_flow;
        bool gyro_integral_valid = std::isfinite(gyro_x) && std::isfinite(gyro_y);
        if (!gyro_integral_valid)
        {
            gyro_x = latest_gyro_.x();
            gyro_y = latest_gyro_.y();
        }
        else if (imu_is_frd_)
        {
            gyro_y = -gyro_y;
        }

        // 从光流恢复机体 FLU 速度 (推导见文件头注释)。
        const double vx_body_raw = distance_meas * (flow_x - gyro_x);
        const double vy_body_raw = distance_meas * (flow_y + gyro_y);
        const double vx_sigma = std::hypot(
            flow_relative_noise_std_ * std::abs(vx_body_raw),
            flow_base_noise_std_);
        const double vy_sigma = std::hypot(
            flow_relative_noise_std_ * std::abs(vy_body_raw),
            flow_base_noise_std_);
        const double vx_noise = vx_sigma * standard_normal_(flow_noise_rng_);
        const double vy_noise = vy_sigma * standard_normal_(flow_noise_rng_);
        const double vx_body = vx_body_raw + vx_noise;
        const double vy_body = vy_body_raw + vy_noise;

        // --- 光流诊断: EKF 预测 body 速度 ---
        Eigen::Matrix3d R_body2world = q_.toRotationMatrix();
        Eigen::Vector3d est_v_body = R_body2world.transpose() * v_;
        double est_vx_body = est_v_body.x();
        double est_vy_body = est_v_body.y();

        // --- 光流诊断: GT body 速度 (最近帧对齐) ---
        double flow_gt_dt = std::numeric_limits<double>::quiet_NaN();
        double gt_vx_body = std::numeric_limits<double>::quiet_NaN();
        double gt_vy_body = std::numeric_limits<double>::quiet_NaN();
        if (has_gt_ && !gt_stamp_.isZero())
        {
            flow_gt_dt = (msg->header.stamp - gt_stamp_).toSec();
            Eigen::Vector3d gt_v_body = gt_quat_.toRotationMatrix().transpose() * gt_vel_;
            gt_vx_body = gt_v_body.x();
            gt_vy_body = gt_v_body.y();
        }

        // --- 光流诊断: IMU 状态时间差 ---
        double flow_imu_dt = has_imu_init_ && !last_imu_time_.isZero()
                                 ? (msg->header.stamp - last_imu_time_).toSec()
                                 : std::numeric_limits<double>::quiet_NaN();

        // ToF 高度观测 (减去初始基准)
        const double pz_meas = height_ref_z_ + distance_meas - height0_;

        // 水平速度和高度分别更新、分别做 NIS 检验。异常光流不再连带
        // 丢弃有效的 ToF 高度约束。
        VelocityUpdateDiagnostics velocity_diag;
        if (std::isfinite(vx_body) && std::isfinite(vy_body))
        {
            velocity_diag = ekfUpdateVelocity(
                vx_body, vy_body, vx_sigma, vy_sigma);
        }
        else
        {
            ROS_WARN_THROTTLE(10.0,
                              "[INS ESKF] optical flow invalid, height-only update: "
                              "flow=(%.3f, %.3f) gyro=(%.3f, %.3f) distance=%.3f quality=%u",
                              flow_x, flow_y, gyro_x, gyro_y, distance_meas, msg->quality);
        }
        const bool height_accepted = ekfUpdateHeight(pz_meas);

        // --- 写入光流诊断 CSV ---
        writeFlowCsv(msg->header.stamp,
                     flow_imu_dt, flow_gt_dt,
                     flow_x, flow_y, gyro_x, gyro_y,
                     msg->distance, tof_noise, distance_meas,
                     vx_body_raw, vy_body_raw,
                     vx_sigma, vy_sigma,
                     vx_noise, vy_noise,
                     vx_body, vy_body,
                     gt_vx_body, gt_vy_body,
                     est_vx_body, est_vy_body,
                     velocity_diag, height_accepted);

        publishEstimate(msg->header.stamp);
    }

    // =====================================================================
    //  名义状态预测
    // =====================================================================

    void predictNominal(const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &gyro,
                        double dt)
    {
        Eigen::Vector3d a_hat = acc - ba_;
        Eigen::Vector3d w_hat = gyro - bg_;
        Eigen::Matrix3d R = q_.toRotationMatrix();

        // 世界系加速度: a_world = R * a_body + g
        Eigen::Vector3d a_world = R * a_hat + g_;

        // 位置/速度
        p_ += v_ * dt + 0.5 * a_world * dt * dt;
        v_ += a_world * dt;

        // 姿态: q = q * Exp(w*dt), 右乘 = 局部(body)系更新
        q_ = (q_ * Exp(w_hat * dt)).normalized();
    }

    // =====================================================================
    //  误差状态协方差传播
    // =====================================================================

    void predictCovariance(const Eigen::Vector3d &acc,
                           const Eigen::Vector3d &gyro,
                           double dt)
    {
        Eigen::Vector3d a_hat = acc - ba_;
        Eigen::Vector3d w_hat = gyro - bg_;
        Eigen::Matrix3d R     = q_.toRotationMatrix();

        // --- Fc (15×15) ---
        Eigen::Matrix<double, 15, 15> Fc;
        Fc.setZero();

        Fc.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();
        Fc.block<3,3>(3, 6) = -R * Skew(a_hat);
        Fc.block<3,3>(3, 9) = -R;
        Fc.block<3,3>(6, 6)  = -Skew(w_hat);
        Fc.block<3,3>(6, 12) = -Eigen::Matrix3d::Identity();
        Fc.block<3,3>(9, 9)   = -(1.0 / tau_ba_) * Eigen::Matrix3d::Identity();
        Fc.block<3,3>(12, 12) = -(1.0 / tau_bg_) * Eigen::Matrix3d::Identity();

        // Fd = I + Fc * dt
        Eigen::Matrix<double, 15, 15> Fd =
            Eigen::Matrix<double, 15, 15>::Identity() + Fc * dt;

        // Match the SDF plugin's exact first-order Gauss-Markov bias transition.
        const double phi_ba = std::exp(-dt / tau_ba_);
        const double phi_bg = std::exp(-dt / tau_bg_);
        const double one_minus_phi_ba = -std::expm1(-dt / tau_ba_);
        const double one_minus_phi_bg = -std::expm1(-dt / tau_bg_);
        const double int_phi_ba = tau_ba_ * one_minus_phi_ba;
        const double int_phi_bg = tau_bg_ * one_minus_phi_bg;
        Fd.block<3,3>(3, 9) =
            -R * int_phi_ba;
        Fd.block<3,3>(6, 12) =
            -int_phi_bg * Eigen::Matrix3d::Identity();
        Fd.block<3,3>(9, 9) =
            phi_ba * Eigen::Matrix3d::Identity();
        Fd.block<3,3>(12, 12) =
            phi_bg * Eigen::Matrix3d::Identity();

        // --- G (15×12) ---
        Eigen::Matrix<double, 15, 12> G;
        G.setZero();

        G.block<3,3>(3, 0)  = -R;
        G.block<3,3>(6, 3)  = -Eigen::Matrix3d::Identity();
        G.block<3,3>(9, 6)  =  Eigen::Matrix3d::Identity();
        G.block<3,3>(12, 9) =  Eigen::Matrix3d::Identity();

        // --- Qc (12×12) ---
        Eigen::Matrix<double, 12, 12> Qc;
        Qc.setZero();

        double sa2  = sigma_acc_  * sigma_acc_;
        double sg2  = sigma_gyro_ * sigma_gyro_;
        double sba2 = sigma_ba_   * sigma_ba_;
        double sbg2 = sigma_bg_   * sigma_bg_;

        Qc.block<3,3>(0, 0) = sa2  * Eigen::Matrix3d::Identity();  // na
        Qc.block<3,3>(3, 3) = sg2  * Eigen::Matrix3d::Identity();  // ng
        Qc.block<3,3>(6, 6) = sba2 * Eigen::Matrix3d::Identity();  // nba
        Qc.block<3,3>(9, 9) = sbg2 * Eigen::Matrix3d::Identity();  // nbg

        Qd_ = G * Qc * G.transpose() * dt;
        const double q_ba =
            sba2 * tau_ba_ * 0.5 * (-std::expm1(-2.0 * dt / tau_ba_));
        const double q_bg =
            sbg2 * tau_bg_ * 0.5 * (-std::expm1(-2.0 * dt / tau_bg_));
        Qd_.block<3,3>(9, 9) =
            q_ba * Eigen::Matrix3d::Identity();
        Qd_.block<3,3>(12, 12) =
            q_bg * Eigen::Matrix3d::Identity();

        // P = Fd * P * Fd^T + Qd
        P_ = Fd * P_ * Fd.transpose() + Qd_;
    }

    // =====================================================================
    //  EKF 观测更新
    // =====================================================================

    /** 世界竖直轴在 body 误差坐标中的方向及其正交投影。
     *
     * IMU + body-frame 平面速度 + 高度对绕世界竖直轴的全局 yaw 不可观。
     * 将姿态和 gyro-bias 校正投影到该方向的正交平面，可防止光流噪声
     * 向不可观 yaw 注入虚假信息。
     */
    Eigen::Matrix3d observableTiltProjector(const Eigen::Matrix3d& R) const
    {
        Eigen::Vector3d yaw_axis_body =
            R.transpose() * Eigen::Vector3d::UnitZ();
        yaw_axis_body.normalize();
        return Eigen::Matrix3d::Identity() -
               yaw_axis_body * yaw_axis_body.transpose();
    }

    void symmetrizeCovariance()
    {
        P_ = 0.5 * (P_ + P_.transpose());
        for (int i = 0; i < P_.rows(); ++i)
        {
            if (!std::isfinite(P_(i, i)) || P_(i, i) < 1e-12)
                P_(i, i) = 1e-12;
        }
    }

    /** 姿态误差注入后，将协方差重置到新的名义状态切空间。 */
    void resetErrorStateCovariance(const Eigen::Vector3d& dtheta)
    {
        if (dtheta.squaredNorm() < 1e-20)
        {
            symmetrizeCovariance();
            return;
        }

        Eigen::Matrix<double, 15, 15> G_reset =
            Eigen::Matrix<double, 15, 15>::Identity();
        G_reset.block<3,3>(6, 6) =
            Eigen::Matrix3d::Identity() - 0.5 * Skew(dtheta);
        P_ = G_reset * P_ * G_reset.transpose();
        symmetrizeCovariance();
    }

    VelocityUpdateDiagnostics ekfUpdateVelocity(
        double vx_body_meas, double vy_body_meas,
        double vx_noise_std, double vy_noise_std)
    {
        VelocityUpdateDiagnostics diag;

        const Eigen::Matrix3d R = q_.toRotationMatrix();
        const Eigen::Vector3d vb = R.transpose() * v_;
        const Eigen::Vector2d z(vx_body_meas, vy_body_meas);
        const Eigen::Vector2d z_hat(vb.x(), vb.y());
        const Eigen::Vector2d y = z - z_hat;
        const double innovation_norm = y.norm();
        const bool exceeds_hard_gate =
            innovation_norm > max_velocity_innovation_;

        if (exceeds_hard_gate)
        {
            ++consecutive_velocity_rejects_;
            if (!velocity_recovery_active_ &&
                consecutive_velocity_rejects_ <
                velocity_recovery_reject_count_)
            {
                ROS_WARN_THROTTLE(
                    5.0,
                    "[INS ESKF] reject velocity update by hard gate: "
                    "|y|=%.2f m/s limit=%.2f reject_streak=%d/%d "
                    "y=(%.2f, %.2f)",
                    innovation_norm, max_velocity_innovation_,
                    consecutive_velocity_rejects_,
                    velocity_recovery_reject_count_,
                    y.x(), y.y());
                return diag;
            }
            velocity_recovery_active_ = true;
            diag.recovery_update = true;
        }
        else if (velocity_recovery_active_)
        {
            velocity_recovery_active_ = false;
            consecutive_velocity_rejects_ = 0;
            ROS_INFO("[INS ESKF] velocity recovery complete: |y|=%.2f m/s",
                     innovation_norm);
        }

        // After global-yaw projection, direct roll/pitch sensitivity is
        // proportional to body vertical velocity. Horizontal speed alone
        // therefore must not enable attitude updates.
        diag.attitude_enabled =
            !disable_att_ &&
            !diag.recovery_update &&
            std::abs(vb.z()) >= attitude_update_min_vertical_speed_ &&
            innovation_norm <= attitude_update_max_innovation_;

        const Eigen::Matrix3d tilt_projector =
            observableTiltProjector(R);

        Eigen::Matrix<double, 2, 15> H;
        H.setZero();
        H.block<2,3>(0, 3) = R.transpose().topRows<2>();

        // δ(R^T v) = Skew(vb) δθ for the right-multiplicative error.
        // Keep only the observable tilt subspace; global yaw is excluded.
        if (diag.attitude_enabled)
        {
            H.block<2,3>(0, 6) =
                Skew(vb).topRows<2>() * tilt_projector;
        }

        Eigen::Matrix2d R_update = Eigen::Matrix2d::Zero();
        R_update(0, 0) = vx_noise_std * vx_noise_std;
        R_update(1, 1) = vy_noise_std * vy_noise_std;

        const Eigen::Matrix2d S =
            H * P_ * H.transpose() + R_update;
        Eigen::LLT<Eigen::Matrix2d> llt(S);
        if (llt.info() != Eigen::Success)
            return diag;

        const Eigen::Matrix2d S_inv =
            llt.solve(Eigen::Matrix2d::Identity());
        if (!S_inv.allFinite())
            return diag;

        diag.nis = (y.transpose() * S_inv * y).value();
        if (!diag.recovery_update &&
            (!std::isfinite(diag.nis) ||
             diag.nis > max_innovation_sigma_ * max_innovation_sigma_))
        {
            ++consecutive_velocity_rejects_;
            ROS_WARN_THROTTLE(
                10.0,
                "[INS ESKF] reject velocity update: NIS=%.2f "
                "reject_streak=%d y=(%.2f, %.2f)",
                diag.nis, consecutive_velocity_rejects_,
                y.x(), y.y());
            return diag;
        }

        Eigen::Matrix<double, 15, 2> K =
            P_ * H.transpose() * S_inv;

        if (diag.recovery_update)
        {
            // Permanent rejection prevents an already-diverged prediction
            // from ever regaining optical-flow lock. After several rejects,
            // apply a clipped velocity-only correction: attitude and both
            // biases remain untouched.
            Eigen::Matrix<double, 15, 2> K_velocity_only =
                Eigen::Matrix<double, 15, 2>::Zero();
            K_velocity_only.block<3,2>(3, 0) =
                K.block<3,2>(3, 0);
            K = K_velocity_only;
            ROS_WARN_THROTTLE(
                2.0,
                "[INS ESKF] velocity recovery update: |y|=%.2f m/s "
                "reject_streak=%d",
                innovation_norm, consecutive_velocity_rejects_);
        }
        else if (diag.attitude_enabled)
        {
            K.block<3,2>(6, 0) =
                tilt_projector * K.block<3,2>(6, 0);
        }
        else
        {
            K.block<3,2>(6, 0).setZero();
        }

        if (!diag.recovery_update && disable_yaw_bias_update_)
        {
            K.block<3,2>(12, 0) =
                tilt_projector * K.block<3,2>(12, 0);
        }

        Eigen::Vector2d innovation_used = y;
        if (diag.recovery_update && innovation_norm > 1e-12)
        {
            innovation_used *=
                max_velocity_innovation_ / innovation_norm;
        }
        Eigen::Matrix<double, 15, 1> dx = K * innovation_used;

        // Limit each correction family by scaling the corresponding Kalman
        // gain rows. The Joseph update then uses the same effective gain,
        // unlike clipping dx after covariance reduction.
        const auto limitCorrection =
            [&K, &innovation_used, &dx](int row, double max_norm)
            {
                const double correction_norm =
                    dx.segment<3>(row).norm();
                if (correction_norm <= max_norm)
                    return false;
                K.block<3,2>(row, 0) *= max_norm / correction_norm;
                dx = K * innovation_used;
                return true;
            };

        const bool position_limited =
            limitCorrection(0, max_position_correction_);
        const bool velocity_limited =
            limitCorrection(
                3, diag.recovery_update
                       ? max_recovery_velocity_correction_
                       : max_velocity_correction_);
        const bool accel_bias_limited =
            limitCorrection(9, max_accel_bias_correction_);
        const bool gyro_bias_limited =
            limitCorrection(12, max_gyro_bias_correction_);
        if (position_limited || velocity_limited ||
            accel_bias_limited || gyro_bias_limited)
        {
            ROS_WARN_THROTTLE(
                5.0,
                "[INS ESKF] state correction limited: p=%d v=%d ba=%d bg=%d",
                position_limited, velocity_limited,
                accel_bias_limited, gyro_bias_limited);
        }

        if (diag.attitude_enabled)
        {
            const double max_correction =
                attitude_update_max_correction_deg_ * M_PI / 180.0;
            const double correction_norm = dx.segment<3>(6).norm();
            if (correction_norm > max_correction)
            {
                const double scale = max_correction / correction_norm;
                K.block<3,2>(6, 0) *= scale;
                dx = K * innovation_used;
                ROS_WARN_THROTTLE(
                    5.0,
                    "[INS ESKF] attitude correction limited: raw=%.2f deg max=%.2f deg",
                    correction_norm * 180.0 / M_PI,
                    attitude_update_max_correction_deg_);
            }
        }

        const Eigen::Vector3d dtheta = dx.segment<3>(6);
        p_  += dx.segment<3>(0);
        v_  += dx.segment<3>(3);
        ba_ += dx.segment<3>(9);
        bg_ += dx.segment<3>(12);
        if (ba_.norm() > max_accel_bias_norm_)
        {
            ba_ *= max_accel_bias_norm_ / ba_.norm();
            ROS_WARN_THROTTLE(
                5.0,
                "[INS ESKF] accelerometer bias norm limited to %.3f m/s^2",
                max_accel_bias_norm_);
        }
        if (bg_.norm() > max_gyro_bias_norm_)
        {
            bg_ *= max_gyro_bias_norm_ / bg_.norm();
            ROS_WARN_THROTTLE(
                5.0,
                "[INS ESKF] gyro bias norm limited to %.4f rad/s",
                max_gyro_bias_norm_);
        }
        if (diag.attitude_enabled)
            q_ = (q_ * Exp(dtheta)).normalized();

        const Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        const Eigen::Matrix<double, 15, 15> I_KH = I15 - K * H;
        P_ = I_KH * P_ * I_KH.transpose() +
             K * R_update * K.transpose();
        resetErrorStateCovariance(dtheta);

        diag.accepted = true;
        diag.dtheta = dtheta;
        if (!diag.recovery_update)
            consecutive_velocity_rejects_ = 0;
        return diag;
    }

    bool ekfUpdateHeight(double pz_meas)
    {
        if (!std::isfinite(pz_meas))
            return false;

        Eigen::Matrix<double, 1, 15> H;
        H.setZero();
        H(0, 2) = 1.0;

        const double y = pz_meas - p_.z();
        const double S =
            (H * P_ * H.transpose())(0, 0) + R_meas_(2, 2);
        if (!std::isfinite(S) || S < 1e-12)
            return false;

        const double nis = y * y / S;
        if (!std::isfinite(nis) ||
            nis > max_innovation_sigma_ * max_innovation_sigma_)
        {
            ROS_WARN_THROTTLE(
                10.0,
                "[INS ESKF] reject height update: NIS=%.2f y=%.2f",
                nis, y);
            return false;
        }

        Eigen::Matrix<double, 15, 1> K = P_ * H.transpose() / S;

        // Height has no direct attitude information. Do not let accumulated
        // cross-correlation rotate the nominal attitude through this scalar
        // update.
        K.segment<3>(6).setZero();
        if (disable_yaw_bias_update_)
        {
            const Eigen::Matrix3d tilt_projector =
                observableTiltProjector(q_.toRotationMatrix());
            K.segment<3>(12) =
                tilt_projector * K.segment<3>(12);
        }

        Eigen::Matrix<double, 15, 1> dx = K * y;
        const auto limitCorrection =
            [&K, y, &dx](int row, double max_norm)
            {
                const double correction_norm =
                    dx.segment<3>(row).norm();
                if (correction_norm <= max_norm)
                    return;
                K.segment<3>(row) *= max_norm / correction_norm;
                dx = K * y;
            };
        limitCorrection(0, max_position_correction_);
        limitCorrection(3, max_velocity_correction_);
        limitCorrection(9, max_accel_bias_correction_);
        limitCorrection(12, max_gyro_bias_correction_);

        p_  += dx.segment<3>(0);
        v_  += dx.segment<3>(3);
        ba_ += dx.segment<3>(9);
        bg_ += dx.segment<3>(12);
        if (ba_.norm() > max_accel_bias_norm_)
            ba_ *= max_accel_bias_norm_ / ba_.norm();
        if (bg_.norm() > max_gyro_bias_norm_)
            bg_ *= max_gyro_bias_norm_ / bg_.norm();

        const Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        const Eigen::Matrix<double, 15, 15> I_KH = I15 - K * H;
        P_ = I_KH * P_ * I_KH.transpose() +
             K * R_meas_(2, 2) * K.transpose();
        symmetrizeCovariance();
        return true;
    }

    // =====================================================================
    //  光流诊断 CSV
    // =====================================================================

    void initFlowCsv()
    {
        if (flow_csv_dir_.empty())
            return;

        boost::system::error_code ec;
        boost::filesystem::create_directories(flow_csv_dir_, ec);
        if (ec)
        {
            ROS_WARN("[INS ESKF] cannot create flow_csv_dir: %s", flow_csv_dir_.c_str());
            return;
        }

        std::string drone_name = ns_;
        if (!drone_name.empty() && drone_name[0] == '/')
            drone_name = drone_name.substr(1);

        std::string path = flow_csv_dir_ + "/" + drone_name + "_flow_diag.csv";
        flow_csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!flow_csv_.is_open())
        {
            ROS_WARN("[INS ESKF] cannot open flow diagnostic CSV: %s", path.c_str());
            return;
        }

        flow_csv_ << std::fixed << std::setprecision(9);
        flow_csv_ << "stamp,"
                  << "flow_imu_dt,flow_gt_dt,"
                  << "flow_x,flow_y,"
                  << "gyro_x,gyro_y,"
                  << "tof_raw,tof_noise,tof_meas,pz_meas,"
                  << "vx_body_raw,vy_body_raw,"
                  << "vx_noise_std,vy_noise_std,"
                  << "vx_noise,vy_noise,"
                  << "vx_body_meas,vy_body_meas,"
                  << "gt_vx_body,gt_vy_body,"
                  << "est_vx_body,est_vy_body,"
                  << "meas_minus_gt_vx,meas_minus_gt_vy,"
                  << "meas_minus_est_vx,meas_minus_est_vy,"
                  << "velocity_update_accepted,velocity_nis,"
                  << "attitude_update_enabled,recovery_update,"
                  << "dtheta_x,dtheta_y,dtheta_z,"
                  << "height_update_accepted\n";
        ROS_INFO("[INS ESKF] flow diagnostic CSV: %s", path.c_str());
    }

    void writeFlowCsv(const ros::Time &stamp,
                      double flow_imu_dt, double flow_gt_dt,
                      double flow_x, double flow_y,
                      double gyro_x, double gyro_y,
                      double tof_raw, double tof_noise, double tof_meas,
                      double vx_body_raw, double vy_body_raw,
                      double vx_noise_std, double vy_noise_std,
                      double vx_noise, double vy_noise,
                      double vx_body, double vy_body,
                      double gt_vx_body, double gt_vy_body,
                      double est_vx_body, double est_vy_body,
                      const VelocityUpdateDiagnostics& velocity_diag,
                      bool height_accepted)
    {
        if (!flow_csv_.is_open())
            return;

        flow_csv_ << stamp.toSec() << ','
                  << flow_imu_dt << ',' << flow_gt_dt << ','
                  << flow_x << ',' << flow_y << ','
                  << gyro_x << ',' << gyro_y << ','
                  << tof_raw << ',' << tof_noise << ',' << tof_meas << ','
                  << (height_ref_z_ + tof_meas - height0_) << ','
                  << vx_body_raw << ',' << vy_body_raw << ','
                  << vx_noise_std << ',' << vy_noise_std << ','
                  << vx_noise << ',' << vy_noise << ','
                  << vx_body << ',' << vy_body << ','
                  << gt_vx_body << ',' << gt_vy_body << ','
                  << est_vx_body << ',' << est_vy_body << ','
                  << (vx_body - gt_vx_body) << ','
                  << (vy_body - gt_vy_body) << ','
                  << (vx_body - est_vx_body) << ','
                  << (vy_body - est_vy_body) << ','
                  << static_cast<int>(velocity_diag.accepted) << ','
                  << velocity_diag.nis << ','
                  << static_cast<int>(velocity_diag.attitude_enabled) << ','
                  << static_cast<int>(velocity_diag.recovery_update) << ','
                  << velocity_diag.dtheta.x() << ','
                  << velocity_diag.dtheta.y() << ','
                  << velocity_diag.dtheta.z() << ','
                  << static_cast<int>(height_accepted) << '\n';
        flow_csv_.flush();
    }

    // =====================================================================
    //  发布 nav_msgs/Odometry
    // =====================================================================

    void publishEstimate(ros::Time stamp)
    {
        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id  = "base_link";

        Eigen::Vector3d p_rel = p_ - p0_;
        msg.pose.pose.position.x = p_rel.x();
        msg.pose.pose.position.y = p_rel.y();
        msg.pose.pose.position.z = p_rel.z();

        // 内部 q_ 表示 body(FLU) -> ENU; 对外保持同一语义发布.
        Eigen::Quaterniond q_pub = q_.normalized();
        msg.pose.pose.orientation.w = q_pub.w();
        msg.pose.pose.orientation.x = q_pub.x();
        msg.pose.pose.orientation.y = q_pub.y();
        msg.pose.pose.orientation.z = q_pub.z();

        msg.twist.twist.linear.x = v_.x();
        msg.twist.twist.linear.y = v_.y();
        msg.twist.twist.linear.z = v_.z();

        // 协方差
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                msg.pose.covariance[r * 6 + c] = publish_pos_cov_scale_ * P_(r, c);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                msg.pose.covariance[(r + 3) * 6 + (c + 3)] = P_(6 + r, 6 + c);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                msg.twist.covariance[r * 6 + c] = publish_vel_cov_scale_ * P_(3 + r, 3 + c);

        odom_pub_.publish(msg);

        // RPY debug output (every 5 seconds when ROS debug logging is enabled)
        static ros::Time last_rpy_print;
        if ((stamp - last_rpy_print).toSec() > 5.0)
        {
            last_rpy_print = stamp;
            Eigen::Vector3d rpy = quatToRPY(q_);
            ROS_DEBUG("[INS ESKF] t=%.1f  RPY=(%.1f, %.1f, %.1f) deg  "
                      "pz=%.2f  vb=(%.2f,%.2f)  ba=(%.3f,%.3f,%.3f)  bg=(%.4f,%.4f,%.4f)",
                      stamp.toSec(), rpy.x(), rpy.y(), rpy.z(),
                      p_.z(),
                      (q_.toRotationMatrix().transpose() * v_).x(),
                      (q_.toRotationMatrix().transpose() * v_).y(),
                      ba_.x(), ba_.y(), ba_.z(),
                      bg_.x(), bg_.y(), bg_.z());
        }
    }

    // =====================================================================
    //  成员变量
    // =====================================================================

    // ROS
    std::string ns_;
    ros::NodeHandle nh_;
    ros::Subscriber odom_init_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber flow_sub_;
    ros::Publisher  odom_pub_;

    // 重力
    Eigen::Vector3d g_;

    // --- 光流诊断 CSV ---
    std::string flow_csv_dir_;
    std::ofstream flow_csv_;

    // --- GT 缓存 (来自 /mavros/local_position/odom) ---
    bool      has_gt_ = false;
    ros::Time gt_stamp_;
    Eigen::Vector3d    gt_pos_{0, 0, 0};
    Eigen::Vector3d    gt_vel_{0, 0, 0};
    Eigen::Quaterniond gt_quat_{1, 0, 0, 0};

    // --- 名义状态 (16维) ---
    Eigen::Vector3d    p_{0, 0, 0};
    Eigen::Vector3d    p0_{0, 0, 0};
    Eigen::Vector3d    v_{0, 0, 0};
    Eigen::Quaterniond q_{1, 0, 0, 0};
    Eigen::Vector3d    ba_{0, 0, 0};
    Eigen::Vector3d    bg_{0, 0, 0};

    // --- 误差状态协方差 (15×15) ---
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Matrix<double, 15, 15> Qd_;  // 当前离散噪声 (发布时不直接用, 保留)

    // --- 噪声参数 ---
    double sigma_acc_;
    double sigma_gyro_;
    double sigma_ba_;
    double sigma_bg_;
    double tau_ba_;
    double tau_bg_;
    double flow_relative_noise_std_ = 0.18;
    double flow_base_noise_std_ = 0.05;
    double tof_noise_std_ = 0.05;
    double tof_min_range_ = 0.20;
    double initial_attitude_std_deg_ = 3.0;
    double attitude_update_min_vertical_speed_ = 0.20;
    double attitude_update_max_innovation_ = 0.15;
    double attitude_update_max_correction_deg_ = 0.15;
    double initial_gyro_bias_std_ = 0.001;
    double max_velocity_innovation_ = 1.20;
    int velocity_recovery_reject_count_ = 5;
    double max_recovery_velocity_correction_ = 0.10;
    double max_position_correction_ = 0.25;
    double max_velocity_correction_ = 0.40;
    double max_accel_bias_correction_ = 0.02;
    double max_gyro_bias_correction_ = 0.0002;
    double max_accel_bias_norm_ = 0.25;
    double max_gyro_bias_norm_ = 0.003;
    int measurement_noise_seed_ = 1;
    std::mt19937 flow_noise_rng_;
    std::normal_distribution<double> standard_normal_{0.0, 1.0};
    Eigen::Matrix3d R_meas_;

    // --- 控制开关 ---
    bool disable_att_ = false;
    bool disable_yaw_bias_update_ = true;
    std::string imu_frame_param_;
    int min_flow_quality_ = 100;
    double max_innovation_sigma_ = 6.0;
    double publish_pos_cov_scale_ = 2.0;
    double publish_vel_cov_scale_ = 10.0;
    int consecutive_velocity_rejects_ = 0;
    bool velocity_recovery_active_ = false;

    // --- 时间 & 初始化 ---
    ros::Time last_imu_time_;
    Eigen::Vector3d latest_gyro_{0, 0, 0};
    bool      odom_inited_        = false;
    bool      has_imu_init_       = false;
    bool      imu_frame_resolved_ = false;
    bool      imu_is_frd_         = false;
    int       imu_acc_samples_    = 0;
    Eigen::Vector3d imu_acc_sum_{0, 0, 0};
    Eigen::Vector3d imu_gyro_sum_{0, 0, 0};
    double    height0_            = 0.0;
    double    height_ref_z_       = 0.0;
    bool      height_initialized_ = false;
};

// =============================================================================
//  main
// =============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ins_eskf");
    InsEkf node;
    ros::spin();
    return 0;
}
