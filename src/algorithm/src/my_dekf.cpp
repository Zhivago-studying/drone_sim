/**
 * @file  my_dekf.cpp
 * @brief 分布式扩展卡尔曼滤波器 (DEKF)
 *
 * 接收 DGO (相对位姿观测)、UWB (测距观测) 和 Camera (相对角度观测) 三个传感器输入，
 * 通过 EKF 融合得到无人机间的相对位姿/速度最优估计。
 *
 * =========================== 节点输入/输出 ===========================
 * 订阅:
 *   /iris_{i}/dgo_estimate          (nav_msgs/Odometry)     — DGO 相对位姿
 *   uwb_processed                    (data_process::UwbProcessed) — UWB 测距
 *   camera_angle_match               (data_process::CameraAngleMatch) — 相机角度
 *
 * 发布:
 *   /iris_{self}/dekf/iris_{target}  (nav_msgs/Odometry)    — DEKF 相对状态
 *
 * =========================== 关键参数 ===========================
 *   rate (30.0)        — DEKF 运行频率 (Hz)
 *   sigma_ax/ay/az     — 过程噪声 (相对加速度)
 *   sigma_px/py/pz     — DGO 位置观测噪声
 *   sigma_uwb          — UWB 测距观测噪声
 *   sigma_alpha/theta  — 相机角度观测噪声
 *   max_observation_delay (0.60)     — 观测最大允许延迟 (s)
 *   current_delay_threshold (0.020)  — "当前时刻"延迟阈值 (s)
 *
 * =========================== 状态机 ===========================
 *   每一 DEKF 周期:
 *     1. 所有滤波器 predict 到当前时间
 *     2. 取出 pending 观测并按时间戳排序
 *     3. 逐条按 obs 年龄分支:
 *        age < current_delay_threshold → applyCurrentUpdate (标准 EKF)
 *        age < max_observation_delay   → applyDelayedUpdate (延迟补偿)
 *        否则                        → 丢弃
 *     4. 发布所有滤波器估计结果
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <data_process/CameraAngleMatch.h>
#include <data_process/UwbProcessed.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// ── 辅助函数 ───────────────────────────────────────────────
double wrapAngle(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

class DEKF
{
public:
    // ── 构造函数 ─────────────────────────────────────────────
    DEKF() : nh_(), pnh_("~")
    {
        pnh_.param("uav_id", uav_id_, 0);
        pnh_.param("uav_num", uav_num_, 4);
        pnh_.param("init_pos_std", init_pos_std_, 0.75);
        pnh_.param("init_vel_std", init_vel_std_, 0.50);
        pnh_.param("sigma_ax", sigma_ax_, 0.80);
        pnh_.param("sigma_ay", sigma_ay_, 0.80);
        pnh_.param("sigma_az", sigma_az_, 0.80);
        pnh_.param("sigma_px", sigma_px_, 0.18);
        pnh_.param("sigma_py", sigma_py_, 0.18);
        pnh_.param("sigma_pz", sigma_pz_, 0.18);
        pnh_.param("sigma_uwb", sigma_uwb_, 0.05);
        pnh_.param("sigma_alpha", sigma_alpha_, 0.05);
        pnh_.param("sigma_theta", sigma_theta_, 0.05);
        pnh_.param("max_dgo_pair_dt", max_dgo_pair_dt_, 0.12);
        pnh_.param("max_observation_delay", max_observation_delay_, 0.60);
        pnh_.param("current_delay_threshold", current_delay_threshold_, 0.020);

        dgo_cache_.resize(uav_num_);

        // 过程噪声矩阵 Q
        Q_.setZero();
        Q_(0, 0) = sigma_ax_ * sigma_ax_;
        Q_(1, 1) = sigma_ay_ * sigma_ay_;
        Q_(2, 2) = sigma_az_ * sigma_az_;

        // 观测噪声矩阵 R (对角: px, py, pz, uwb, alpha, theta)
        R_.setZero();
        R_(0, 0) = sigma_px_ * sigma_px_;
        R_(1, 1) = sigma_py_ * sigma_py_;
        R_(2, 2) = sigma_pz_ * sigma_pz_;
        R_(3, 3) = sigma_uwb_ * sigma_uwb_;
        R_(4, 4) = sigma_alpha_ * sigma_alpha_;
        R_(5, 5) = sigma_theta_ * sigma_theta_;

        // 初始化滤波器: 每个 target 一个, 跳过本机
        filters_.resize(uav_num_);
        for (int i = 0; i < uav_num_; ++i)
        {
            filters_[i].target_id = i;
            if (i == uav_id_)
                continue;
            filters_[i].X.setZero();
            filters_[i].P.setZero();
            filters_[i].P.block<3, 3>(0, 0).diagonal().setConstant(0.75 * 0.75);
            filters_[i].P.block<3, 3>(3, 3).diagonal().setConstant(0.50 * 0.50);

            std::string topic = "/iris_" + std::to_string(uav_id_) +
                                "/dekf/iris_" + std::to_string(i);
            filters_[i].pub = nh_.advertise<nav_msgs::Odometry>(topic, 10);
        }

        // DGO 订阅
        for (int i = 0; i < uav_num_; ++i)
        {
            std::string topic = "/iris_" + std::to_string(i) + "/dgo_estimate";
            dgo_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    topic, 10,
                    boost::bind(&DEKF::dgoCallback, this, _1, i)));
        }

        // UWB / Camera 订阅
        uwb_sub_ = nh_.subscribe<data_process::UwbProcessed>(
            "uwb_processed", 10, &DEKF::uwbCallback, this);
        camera_sub_ = nh_.subscribe<data_process::CameraAngleMatch>(
            "camera_angle_match", 10, &DEKF::cameraCallback, this);

        // DEKF 定时器
        pnh_.param("rate", rate_, 30.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_),
                                 &DEKF::dekfCallback, this);

        // 初始化滤波器时间戳
        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_)
                continue;
            Filter &f = filters_[i];
            ros::Time now = ros::Time::now();
            initFilter(f, now);
        }
    }

private:
    // ═══════════════════════════════════════════════════════════
    //  类型定义
    // ═══════════════════════════════════════════════════════════

    // 预测历史缓存 (用于延迟补偿重传播)
    struct History_Record
    {
        ros::Time stamp;         // 记录时间戳
        Vector6d X_pred;         // X_{k+1|k}
        Matrix6d P_pred;         // P_{k+1|k}
        Matrix6d A;              // 状态转移矩阵
        Matrix6d I_KH;           // I - K*H  (更新后的投影矩阵)
    };

    // 观测类型枚举
    enum class ObsType
    {
        DGO,             // DGO 相对位姿观测
        UWB_RANGE,       // UWB 测距观测
        CAMERA_BEARING   // 相机角度观测
    };

    // 单条观测结构 (由 callback 产生, 由 dekfCallback 消费)
    struct Observation
    {
        ObsType type;
        int target_id = -1;
        ros::Time stamp;
        double value = 0.0;                      // UWB 测距值
        Eigen::Vector3d position = Eigen::Vector3d::Zero();  // DGO 相对位置
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // DGO 相对速度
        double alpha = 0.0;
        double theta = 0.0;
        bool has_alpha = false;
        bool has_theta = false;
    };

    // 滤波器: 本机到 target_id 的相对位姿/速度估计
    struct Filter
    {
        int target_id = -1;
        bool initialized = false;
        ros::Time stamp;            // 当前预测时间戳
        Vector6d X = Vector6d::Zero();   // [px, py, pz, vx, vy, vz]
        Matrix6d P = Matrix6d::Identity();  // 协方差矩阵

        std::deque<History_Record> cache;   // 历史缓存
        std::deque<Observation> pending_obs; // 待处理观测队列
        ros::Publisher pub;                  // 结果发布器
    };

    // DGO 历史观测 (按 UAV 编号缓存)
    struct DGOSample
    {
        ros::Time stamp;
        nav_msgs::Odometry msg;
    };

    // ═══════════════════════════════════════════════════════════
    //  成员变量
    // ═══════════════════════════════════════════════════════════
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // DEKF 定时 / 无人机 ID
    ros::Timer timer_;
    double rate_ = 30.0;
    int uav_id_ = 0;
    int uav_num_ = 4;

    // 噪声参数 (过程噪声)
    double sigma_ax_ = 0.80;
    double sigma_ay_ = 0.80;
    double sigma_az_ = 0.80;
    double init_pos_std_ = 0.75;
    double init_vel_std_ = 0.50;
    Eigen::Matrix3d Q_;

    // 噪声参数 (观测噪声)
    double sigma_px_ = 0.18;
    double sigma_py_ = 0.18;
    double sigma_pz_ = 0.18;
    double sigma_uwb_ = 0.05;
    double sigma_alpha_ = 0.05;
    double sigma_theta_ = 0.05;
    Matrix6d R_;

    // 延迟补偿参数
    double max_dgo_pair_dt_ = 0.12;
    double max_observation_delay_ = 0.60;
    double current_delay_threshold_ = 0.020;
    ros::Time last_predict_stamp_;

    // 订阅器 & 数据缓存
    std::vector<ros::Subscriber> dgo_subs_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;
    std::vector<Filter> filters_;
    std::vector<std::deque<DGOSample>> dgo_cache_;

    // ═══════════════════════════════════════════════════════════
    //  观测回调函数
    // ═══════════════════════════════════════════════════════════

    // DGO 回调: 收到 /iris_{id}/dgo_estimate 后, 与已缓存的自身 DGO 配对生成观测
    void dgoCallback(const nav_msgs::Odometry::ConstPtr &msg, int id)
    {
        DGOSample sample;
        sample.msg = *msg;
        sample.stamp = msg->header.stamp;
        dgo_cache_[id].push_back(sample);
        if (dgo_cache_[id].size() > 100)
            dgo_cache_[id].pop_front();

        const int peer_id = (id == uav_id_) ? -1 : id;
        const int self_id = uav_id_;

        if (dgo_cache_[self_id].empty())
            return;

        // 如果刚收到的是 self 数据, 遍历所有 peer 配对
        // 如果刚收到的是 peer 数据, 只与 self 配对
        int start = (id == uav_id_) ? 0 : peer_id;
        int end   = (id == uav_id_) ? uav_num_ : peer_id + 1;

        for (int target = start; target < end; ++target)
        {
            if (target == self_id || dgo_cache_[target].empty())
                continue;

            const DGOSample &self = dgo_cache_[self_id].back();
            const DGOSample &peer = dgo_cache_[target].back();
            if (std::abs((self.stamp - peer.stamp).toSec()) > max_dgo_pair_dt_)
                continue;

            ros::Time obs_stamp = self.stamp < peer.stamp ? self.stamp : peer.stamp;

            Observation obs;
            obs.type = ObsType::DGO;
            obs.target_id = target;
            obs.stamp = obs_stamp;
            obs.position.x() = peer.msg.pose.pose.position.x - self.msg.pose.pose.position.x;
            obs.position.y() = peer.msg.pose.pose.position.y - self.msg.pose.pose.position.y;
            obs.position.z() = peer.msg.pose.pose.position.z - self.msg.pose.pose.position.z;
            obs.velocity.x() = peer.msg.twist.twist.linear.x - self.msg.twist.twist.linear.x;
            obs.velocity.y() = peer.msg.twist.twist.linear.y - self.msg.twist.twist.linear.y;
            obs.velocity.z() = peer.msg.twist.twist.linear.z - self.msg.twist.twist.linear.z;

            if (!obs.position.allFinite() || !obs.velocity.allFinite())
                continue;

            filters_[target].pending_obs.push_back(obs);
        }
    }

    // UWB 回调: 收到 uwb_processed 后, 为每个 target 生成测距观测
    void uwbCallback(const data_process::UwbProcessed::ConstPtr &msg)
    {
        for (size_t i = 0; i < msg->target_ids.size() && i < msg->distances.size(); ++i)
        {
            int target = msg->target_ids[i];
            if (target < 0 || target >= uav_num_ || target == uav_id_)
                continue;
            if (!std::isfinite(msg->distances[i]) || msg->distances[i] < 0.05)
                continue;

            Observation obs;
            obs.type = ObsType::UWB_RANGE;
            obs.target_id = target;
            obs.stamp = msg->header.stamp;
            obs.value = msg->distances[i];
            filters_[target].pending_obs.push_back(obs);
        }
    }

    // Camera 回调: 收到 camera_angle_match 后, 为每个 target 生成角度观测
    void cameraCallback(const data_process::CameraAngleMatch::ConstPtr &msg)
    {
        size_t n = std::min(msg->id.size(), std::min(msg->alpha.size(), msg->theta.size()));
        for (size_t i = 0; i < n; ++i)
        {
            int target = msg->id[i];
            if (target < 0 || target >= uav_num_ || target == uav_id_)
                continue;

            Observation obs;
            obs.type = ObsType::CAMERA_BEARING;
            obs.target_id = target;
            obs.stamp = msg->header.stamp;
            obs.has_alpha = std::isfinite(msg->alpha[i]);
            obs.has_theta = std::isfinite(msg->theta[i]);
            if (obs.has_alpha)
                obs.alpha = msg->alpha[i];
            if (obs.has_theta)
                obs.theta = msg->theta[i];
            if (!obs.has_alpha && !obs.has_theta)
                continue;

            filters_[target].pending_obs.push_back(obs);
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  DEKF 主循环: 预测 → 观测更新 → 发布
    // ═══════════════════════════════════════════════════════════

    // 每个 DEKF 周期执行一次
    void dekfCallback(const ros::TimerEvent &event)
    {
        (void)event;
        const ros::Time now = ros::Time::now();

        // 1. 所有 filter predict 到 now
        for (int target = 0; target < uav_num_; ++target)
        {
            if (target == uav_id_)
                continue;
            Filter &f = filters_[target];

            // 执行预测到当前时间
            predict(f, now);

            // 2. 取出该 filter 的 pending observations
            std::vector<Observation> batch;
            batch.reserve(f.pending_obs.size());
            while (!f.pending_obs.empty())
            {
                batch.push_back(f.pending_obs.front());
                f.pending_obs.pop_front();
            }

            // 3. 按 obs.stamp 排序
            std::sort(batch.begin(), batch.end(),
                    [](const Observation &a, const Observation &b) {
                        return a.stamp < b.stamp;
                    });

            // 4. 逐条观测按年龄分支处理
            for (const Observation &obs : batch)
            {
                if (obs.target_id < 0 || obs.target_id >= uav_num_ || obs.target_id == uav_id_)
                    continue;

                const double age = (f.stamp - obs.stamp).toSec();

                if (age < current_delay_threshold_)
                {
                    // 观测时间 ≈ 当前时间 → 直接在当前状态上更新
                    applyCurrentUpdate(f, obs);
                }
                else if (age < max_observation_delay_)
                {
                    // TODO: applyDelayedUpdate(f, obs) — 由用户实现
                }
                // else: 观测太旧, 直接丢弃
            }

            // 5. 发布该 filter 的 DEKF 估计结果
            publishFilter(f);
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  滤波器核心函数
    // ═══════════════════════════════════════════════════════════

    // 初始化滤波器: 设置初值、协方差、时间戳, 写入首条 cache
    void initFilter(Filter &f, const ros::Time &stamp)
    {
        if (f.initialized)
            return;

        f.X.setZero();
        f.P.setZero();
        f.P.block<3, 3>(0, 0).diagonal().setConstant(
            init_pos_std_ * init_pos_std_);
        f.P.block<3, 3>(3, 3).diagonal().setConstant(
            init_vel_std_ * init_vel_std_);
        f.stamp = stamp;
        f.initialized = true;

        History_Record rec;
        rec.stamp = stamp;
        rec.X_pred = f.X;
        rec.P_pred = f.P;
        rec.A.setIdentity();
        rec.I_KH.setIdentity();
        f.cache.push_back(rec);
    }

    // 预测: 从当前 f.stamp 递推到 stamp (恒速运动模型)
    void predict(Filter &f, const ros::Time &stamp)
    {
        if (!f.initialized)
        {
            initFilter(f, stamp);
            return;
        }

        double dt = (stamp - f.stamp).toSec();
        if (dt < 1e-6)
            return;

        // 构造状态转移矩阵 A (恒速: px+=vx*dt, vx 不变)
        History_Record rec;
        rec.stamp = stamp;
        rec.A.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        rec.A.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        rec.A.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
        rec.A.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

        // 构造噪声耦合矩阵 G
        Eigen::Matrix<double,6,3> G;
        G.block<3,3>(0,0) = 0.5 * Eigen::Matrix3d::Identity() * dt * dt;
        G.block<3,3>(3,0) = Eigen::Matrix3d::Identity() * dt;

        // X_{k+1|k} = A_k * X_k
        rec.X_pred = rec.A * f.X;
        // P_{k+1|k} = A_k * P_k * A_k^T + G_k * Q * G_k^T
        rec.P_pred = rec.A * f.P * rec.A.transpose() + G * Q_ * G.transpose();

        f.X = rec.X_pred;
        f.P = rec.P_pred;
        f.stamp = stamp;

        f.cache.push_back(rec);
        CleanCacheOverflow(f);
    }

    // 清理过期的预测缓存, 控制内存增长
    void CleanCacheOverflow(Filter &f)
    {
        constexpr size_t kMaxCacheSize = 500;
        while (f.cache.size() > kMaxCacheSize)
            f.cache.pop_front();
    }

    // ═══════════════════════════════════════════════════════════
    //  观测模型构建: 根据状态 x 和观测类型, 填充 Z, residual, H, R_meas
    // ═══════════════════════════════════════════════════════════

    bool buildObsModel(const Vector6d &x,
                       const Observation &obs,
                       Eigen::VectorXd &Z,
                       Eigen::VectorXd &residual,
                       Eigen::MatrixXd &H,
                       Eigen::MatrixXd &R_meas)
    {
        switch (obs.type)
        {
            case ObsType::DGO:
            {
                Z.resize(3);
                residual.resize(3);
                Z << obs.position(0), obs.position(1), obs.position(2);
                residual = Z - x.segment(0, 3);
                H.resize(3, 6);
                H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
                H.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
                R_meas.resize(3, 3);
                R_meas = R_.block(0, 0, 3, 3);
                return true;
            }

            case ObsType::UWB_RANGE:
            {
                Z.resize(1);
                residual.resize(1);
                Z << obs.value;
                {
                    double px = x(0), py = x(1), pz = x(2);
                    double r = sqrt(px*px + py*py + pz*pz);
                    residual(0) = Z(0) - r;
                    H.resize(1, 6);
                    H << px/r, py/r, pz/r, 0.0, 0.0, 0.0;
                }
                R_meas.resize(1, 1);
                R_meas = R_.block(3, 3, 1, 1);
                return true;
            }

            case ObsType::CAMERA_BEARING:
            {
                double px = x(0), py = x(1), pz = x(2);
                double r2 = px*px + py*py + pz*pz;
                double r = sqrt(r2);
                double l = sqrt(px*px + py*py);
                double alpha_pred = std::atan2(py, px);
                double theta_pred = std::asin(pz / r);

                if (obs.has_alpha && obs.has_theta)
                {
                    Z.resize(2);
                    residual.resize(2);
                    Z << obs.alpha, obs.theta;
                    residual(0) = wrapAngle(obs.alpha - alpha_pred);
                    residual(1) = wrapAngle(obs.theta - theta_pred);
                    H.resize(2, 6);
                    H.block(0,0,1,6) << -py/(l*l), px/(l*l), 0.0, 0.0, 0.0, 0.0;
                    H.block(1,0,1,6) << -px*pz/(r2*l), -py*pz/(r2*l), l/r2, 0.0, 0.0, 0.0;
                    R_meas.resize(2, 2);
                    R_meas = R_.block(4, 4, 2, 2);
                }
                else if (obs.has_alpha)
                {
                    Z.resize(1);
                    residual.resize(1);
                    Z << obs.alpha;
                    residual(0) = wrapAngle(obs.alpha - alpha_pred);
                    H.resize(1, 6);
                    H.block(0,0,1,6) << -py/(l*l), px/(l*l), 0.0, 0.0, 0.0, 0.0;
                    R_meas.resize(1, 1);
                    R_meas = R_.block(4, 4, 1, 1);
                }
                else if (obs.has_theta)
                {
                    Z.resize(1);
                    residual.resize(1);
                    Z << obs.theta;
                    residual(0) = wrapAngle(obs.theta - theta_pred);
                    H.resize(1, 6);
                    H.block(0,0,1,6) << -px*pz/(r2*l), -py*pz/(r2*l), l/r2, 0.0, 0.0, 0.0;
                    R_meas.resize(1, 1);
                    R_meas = R_.block(5, 5, 1, 1);
                }
                else
                {
                    return false;
                }
                return true;
            }
        }
        return false;
    }

    // ═══════════════════════════════════════════════════════════
    //  观测更新函数
    // ═══════════════════════════════════════════════════════════

    // 当前时刻 EKF 更新: 观测时间 ≈ 当前预测时间, 直接执行标准 EKF 更新
    void applyCurrentUpdate(Filter &f, const Observation &obs)
    {
        Eigen::VectorXd Z;
        Eigen::VectorXd residual;
        Eigen::MatrixXd H;
        Eigen::MatrixXd R_meas;

        if (!buildObsModel(f.X, obs, Z, residual, H, R_meas))
            return;

        // 标准卡尔曼增益: K = P * H^T * (H * P * H^T + R)^{-1}
        Eigen::MatrixXd K = f.P * H * (H * f.P * H.transpose() + R_meas).inverse();
        f.X += K * residual;

        // Joseph 形式协方差更新 (数值稳定性优于标准形式)
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
        f.P = I_KH * f.P * I_KH.transpose() + K * R_meas * K.transpose();

        // 缓存 I_KH 到最新的历史记录 (用于延迟补偿重传播)
        if (!f.cache.empty())
            f.cache.back().I_KH = I_KH;
    }

    // ═══════════════════════════════════════════════════════════
    //  发布函数
    // ═══════════════════════════════════════════════════════════

    // 发布 DEKF 估计结果: /iris_{self}/dekf/iris_{target}
    void publishFilter(const Filter &f)
    {
        if (!f.initialized)
            return;

        nav_msgs::Odometry msg;
        msg.header.stamp = f.stamp;
        msg.header.frame_id = "iris_" + std::to_string(uav_id_) + "/initial_enu";
        msg.child_frame_id = "iris_" + std::to_string(f.target_id) + "/relative";

        msg.pose.pose.orientation.w = 1.0;

        msg.pose.pose.position.x = f.X[0];
        msg.pose.pose.position.y = f.X[1];
        msg.pose.pose.position.z = f.X[2];

        msg.twist.twist.linear.x = f.X[3];
        msg.twist.twist.linear.y = f.X[4];
        msg.twist.twist.linear.z = f.X[5];

        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                msg.pose.covariance[r * 6 + c] = f.P(r, c);
                msg.twist.covariance[r * 6 + c] = f.P(r + 3, c + 3);
            }
        }

        f.pub.publish(msg);
    }
};

// ═══════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_dekf");
    DEKF node;
    ros::spin();
    return 0;
}
