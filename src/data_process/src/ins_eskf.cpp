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
 * 光流恢复 (mockup 用 FRD, 此处推导为 FLU 公式):
 *   mockup FRD: flow_x = w_x_frd - vy_frd/h
 *               flow_y = w_y_frd + vx_frd/h
 *   FRD→FLU: w_x_frd=w_x_flu, w_y_frd=-w_y_flu, vx_frd=vx_flu, vy_frd=-vy_flu
 *   代入解得:  vx_flu = h*(flow_y + w_y_flu)
 *              vy_flu = h*(flow_x - w_x_flu)
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

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
        nh_.param("publish_pos_cov_scale", publish_pos_cov_scale_, 5.0);
        nh_.param("publish_vel_cov_scale", publish_vel_cov_scale_, 1.0);

        // --- IMU 噪声参数 (Gazebo SITL) ---
        sigma_acc_  = 0.05;
        sigma_gyro_ = 0.003;
        sigma_ba_   = 0.003;
        sigma_bg_   = 0.0002;

        // --- 观测噪声 (3×3) ---
        R_meas_.setZero();
        R_meas_(0,0) = 0.35 * 0.35; // 光流 vx
        R_meas_(1,1) = 0.35 * 0.35; // 光流 vy
        R_meas_(2,2) = 0.10 * 0.10; // ToF 高度

        // --- 重力 (ENU) ---
        g_ << 0.0, 0.0, -9.81;

        // --- 初始化状态 ---
        p_.setZero();
        v_.setZero();
        q_.setIdentity();
        ba_.setZero();
        bg_.setZero();

        // 初始协方差 (姿态初始化为 unit quaternion, 不确定性较大)
        P_ = Eigen::Matrix<double, 15, 15>::Identity();
        P_.block<3,3>(0, 0) *= 4.0;     // δp
        P_.block<3,3>(3, 3) *= 2.0;     // δv
        P_.block<3,3>(6, 6) *= 1.0;     // δθ
        P_.block<3,3>(9, 9) *= 0.01;    // δba
        P_.block<3,3>(12,12) *= 0.01;   // δbg

        ROS_INFO("[INS ESKF] ns=%s sigma_acc=%.2e sigma_gyro=%.2e disable_att_update=%d imu_frame=%s",
                 ns_.c_str(), sigma_acc_, sigma_gyro_, disable_att_, imu_frame_param_.c_str());

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

private:
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
            v_.x() = msg->twist.twist.linear.x;
            v_.y() = msg->twist.twist.linear.y;
            v_.z() = msg->twist.twist.linear.z;

            q_.w() = msg->pose.pose.orientation.w;
            q_.x() = msg->pose.pose.orientation.x;
            q_.y() = msg->pose.pose.orientation.y;
            q_.z() = msg->pose.pose.orientation.z;
            odom_inited_ = true;

            Eigen::Vector3d rpy = quatToRPY(q_);
            ROS_INFO("[INS ESKF] state initialized from odom: p=(%.2f, %.2f, %.2f) v=(%.2f, %.2f, %.2f) "
                     "q=(%.3f, %.3f, %.3f, %.3f) RPY(deg)=(%.1f, %.1f, %.1f)",
                     p_.x(), p_.y(), p_.z(), v_.x(), v_.y(), v_.z(),
                     q_.w(), q_.x(), q_.y(), q_.z(), rpy.x(), rpy.y(), rpy.z());
        }
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
            ROS_INFO("[INS ESKF] IMU initialized at t=%.3f", last_imu_time_.toSec());
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

        if (!height_initialized_)
        {
            height0_ = msg->distance;
            height_ref_z_ = p_.z();
            height_initialized_ = true;
            ROS_INFO("[INS ESKF] height0 = %.3f m, height_ref_z = %.3f m",
                     height0_, height_ref_z_);
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

        // 从光流恢复机体 FLU 速度 (推导见文件头注释):
        //   vx_flu = h*(flow_y + omega_y_flu)
        //   vy_flu = h*(flow_x - omega_x_flu)
        double vx_body = msg->distance * (flow_y + gyro_y);
        double vy_body = msg->distance * (flow_x - gyro_x);

        // ToF 高度观测 (减去初始基准)
        double pz_meas = height_ref_z_ + msg->distance - height0_;

        // --- EKF 更新 ---
        if (std::isfinite(vx_body) && std::isfinite(vy_body))
        {
            ekfUpdate(vx_body, vy_body, pz_meas, msg->header.stamp);
        }
        else
        {
            ROS_WARN_THROTTLE(2.0,
                              "[INS ESKF] optical flow invalid, height-only update: "
                              "flow=(%.3f, %.3f) gyro=(%.3f, %.3f) distance=%.3f quality=%u",
                              flow_x, flow_y, gyro_x, gyro_y, msg->distance, msg->quality);
            ekfUpdateHeight(pz_meas, msg->header.stamp);
        }
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

        // Fd = I + Fc * dt
        Eigen::Matrix<double, 15, 15> Fd =
            Eigen::Matrix<double, 15, 15>::Identity() + Fc * dt;

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

        // P = Fd * P * Fd^T + Qd
        P_ = Fd * P_ * Fd.transpose() + Qd_;
    }

    // =====================================================================
    //  EKF 观测更新 (光流 vx/vy + ToF pz)
    // =====================================================================

    void ekfUpdate(double vx_body_meas, double vy_body_meas, double pz_meas,
                   ros::Time stamp)
    {
        Eigen::Matrix3d R = q_.toRotationMatrix();

        // vb = R^T * v: 预测的机体 FLU 速度 (前/左/上)
        Eigen::Vector3d vb = R.transpose() * v_;

        // 预测观测
        Eigen::Vector3d z_hat(vb.x(), vb.y(), p_.z());

        // 实际观测
        Eigen::Vector3d z(vx_body_meas, vy_body_meas, pz_meas);

        // 残差
        Eigen::Vector3d y = z - z_hat;

        // --- H 矩阵 (3×15) ---
        Eigen::Matrix<double, 3, 15> H;
        H.setZero();

        // δ(R^T v) = (R^T v)^× δθ = Skew(vb) δθ
        // 推导: R = R_hat * Exp(δθ) → R^T = Exp(-δθ) * R_hat^T ≈ (I - Skew(δθ)) * R_hat^T
        //   R^T v ≈ vb_hat - δθ × vb_hat = vb_hat + vb_hat × δθ = vb_hat + Skew(vb_hat) δθ
        H.block<1,3>(0, 3) = R.transpose().row(0);

        H.block<1,3>(1, 3) = R.transpose().row(1);

        if (!disable_att_)
        {
            H.block<1,3>(0, 6) = Skew(vb).row(0);
            H.block<1,3>(1, 6) = Skew(vb).row(1);
        }

        H(2, 2) = 1.0;

        // --- 卡尔曼增益 ---
        Eigen::Matrix<double, 3, 3> S =
            H * P_ * H.transpose() + R_meas_;
        if (!std::isfinite(S.determinant()) || S.determinant() < 1e-12)
            return;

        Eigen::Matrix3d S_inv = S.inverse();
        Eigen::Matrix<double, 15, 3> K =
            P_ * H.transpose() * S_inv;

        double nis = (y.transpose() * S_inv * y).value();
        if (!std::isfinite(nis) || nis > max_innovation_sigma_ * max_innovation_sigma_)
        {
            ROS_WARN_THROTTLE(2.0, "[INS ESKF] reject update: NIS=%.2f y=(%.2f, %.2f, %.2f)",
                              nis, y.x(), y.y(), y.z());
            return;
        }

        if (disable_att_)
        {
            K.block<3,3>(6, 0).setZero();
            K.block<3,3>(12, 0).setZero();
        }

        // --- 误差状态 ---
        Eigen::Matrix<double, 15, 1> dx = K * y;

        Eigen::Vector3d dp     = dx.segment<3>(0);
        Eigen::Vector3d dv     = dx.segment<3>(3);
        Eigen::Vector3d dtheta = dx.segment<3>(6);
        Eigen::Vector3d dba    = dx.segment<3>(9);
        Eigen::Vector3d dbg    = dx.segment<3>(12);

        // --- 注入名义状态 ---
        p_  += dp;
        v_  += dv;
        ba_ += dba;
        bg_ += dbg;

        // 姿态更新 (可关闭以隔离实验)
        if (!disable_att_)
        {
            q_ = (q_ * Exp(dtheta)).normalized();
        }

        // --- Joseph 形式协方差更新 ---
        Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        Eigen::Matrix<double, 15, 15> I_KH = I15 - K * H;

        P_ = I_KH * P_ * I_KH.transpose() +
             K * R_meas_ * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        // --- 发布 ---
        publishEstimate(stamp);
    }

    void ekfUpdateHeight(double pz_meas, ros::Time stamp)
    {
        if (!std::isfinite(pz_meas))
            return;

        Eigen::Matrix<double, 1, 15> H;
        H.setZero();
        H(0, 2) = 1.0;

        double y = pz_meas - p_.z();
        double S = (H * P_ * H.transpose())(0, 0) + R_meas_(2, 2);
        if (!std::isfinite(S) || S < 1e-12)
            return;

        double nis = y * y / S;
        if (!std::isfinite(nis) || nis > max_innovation_sigma_ * max_innovation_sigma_)
        {
            ROS_WARN_THROTTLE(2.0, "[INS ESKF] reject height update: NIS=%.2f y=%.2f",
                              nis, y);
            return;
        }

        Eigen::Matrix<double, 15, 1> K = P_ * H.transpose() / S;
        if (disable_att_)
        {
            K.segment<3>(6).setZero();
            K.segment<3>(12).setZero();
        }

        Eigen::Matrix<double, 15, 1> dx = K * y;
        p_  += dx.segment<3>(0);
        v_  += dx.segment<3>(3);
        ba_ += dx.segment<3>(9);
        bg_ += dx.segment<3>(12);
        if (!disable_att_)
        {
            q_ = (q_ * Exp(dx.segment<3>(6))).normalized();
        }

        Eigen::Matrix<double, 15, 15> I15 =
            Eigen::Matrix<double, 15, 15>::Identity();
        Eigen::Matrix<double, 15, 15> I_KH = I15 - K * H;
        P_ = I_KH * P_ * I_KH.transpose() +
             K * R_meas_(2, 2) * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        publishEstimate(stamp);
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

        msg.pose.pose.position.x = p_.x();
        msg.pose.pose.position.y = p_.y();
        msg.pose.pose.position.z = p_.z();

        msg.pose.pose.orientation.w = q_.w();
        msg.pose.pose.orientation.x = q_.x();
        msg.pose.pose.orientation.y = q_.y();
        msg.pose.pose.orientation.z = q_.z();

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

        // RPY 调试输出 (每 5 秒)
        static ros::Time last_rpy_print;
        if ((stamp - last_rpy_print).toSec() > 5.0)
        {
            last_rpy_print = stamp;
            Eigen::Vector3d rpy = quatToRPY(q_);
            ROS_INFO("[INS ESKF] t=%.1f  RPY=(%.1f, %.1f, %.1f) deg  "
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

    // --- 名义状态 (16维) ---
    Eigen::Vector3d    p_{0, 0, 0};
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
    Eigen::Matrix3d R_meas_;

    // --- 控制开关 ---
    bool disable_att_ = false;
    std::string imu_frame_param_;
    int min_flow_quality_ = 100;
    double max_innovation_sigma_ = 6.0;
    double publish_pos_cov_scale_ = 5.0;
    double publish_vel_cov_scale_ = 1.0;

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
