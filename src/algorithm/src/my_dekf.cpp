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
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
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
        pnh_.param("rate", rate_, 30.0);
        pnh_.param("initial_spacing", initial_spacing_, 2.0);
        pnh_.param("csv_dir", csv_dir_, std::string(""));
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
        pnh_.param("max_history_match_dt", max_history_match_dt_, 0.06);
        pnh_.param("future_tolerance", future_tolerance_, 0.02);
        pnh_.param("max_nis", max_nis_, 25.0);
        pnh_.param("max_nis_dgo", max_nis_dgo_, 16.3);
        pnh_.param("max_nis_uwb", max_nis_uwb_, 9.0);
        pnh_.param("max_nis_camera_1d", max_nis_camera_1d_, 9.0);
        pnh_.param("max_nis_camera_2d", max_nis_camera_2d_, 13.8);
        pnh_.param("uwb_residual_gate", uwb_residual_gate_, 0.8);
        pnh_.param("dgo_residual_gate", dgo_residual_gate_, 2.0);
        pnh_.param("camera_residual_gate", camera_residual_gate_, 0.5);
        pnh_.param("require_direction_anchor_for_uwb", require_direction_anchor_for_uwb_, true);
        pnh_.param("uwb_anchor_max_age", uwb_anchor_max_age_, 0.50);
        pnh_.param("enable_dgo_recovery", enable_dgo_recovery_, true);
        pnh_.param("dgo_recovery_gate", dgo_recovery_gate_, 1.0);
        pnh_.param("dgo_recovery_count_threshold", dgo_recovery_count_threshold_, 3);
        pnh_.param("dgo_recovery_consistency_gate", dgo_recovery_consistency_gate_, 0.50);
        pnh_.param("dgo_recovery_pos_std", dgo_recovery_pos_std_, 0.20);
        pnh_.param("dgo_recovery_vel_std", dgo_recovery_vel_std_, 0.50);
        pnh_.param("clear_history_on_recovery", clear_history_on_recovery_, true);
        pnh_.param("max_pending", max_pending_, 100);
        pnh_.param("history_keep_time", history_keep_time_, 2.0);

        // ── 参数合法性检查 ──────────────────────────────────
        {
            bool ok = true;
            if (uav_num_ < 2 || uav_id_ < 0 || uav_id_ >= uav_num_)
            {
                ROS_FATAL("[DEKF] invalid uav_id=%d uav_num=%d", uav_id_, uav_num_);
                ok = false;
            }
            if (uav_num_ != 4)
            {
                ROS_FATAL("[DEKF] current initial_offsets layout supports exactly 4 UAVs, got uav_num=%d",
                          uav_num_);
                ok = false;
            }
            if (rate_ <= 0.0)
            {
                ROS_FATAL("[DEKF] rate=%.2f must be > 0", rate_);
                ok = false;
            }
            if (initial_spacing_ <= 0.0)
            {
                ROS_FATAL("[DEKF] initial_spacing=%.3f must be > 0", initial_spacing_);
                ok = false;
            }
            if (init_pos_std_ <= 0.0 || init_vel_std_ <= 0.0 ||
                sigma_ax_ <= 0.0 || sigma_ay_ <= 0.0 || sigma_az_ <= 0.0)
            {
                ROS_FATAL("[DEKF] process noise sigma must be > 0");
                ok = false;
            }
            if (sigma_px_ <= 0.0 || sigma_py_ <= 0.0 || sigma_pz_ <= 0.0 ||
                sigma_uwb_ <= 0.0 || sigma_alpha_ <= 0.0 || sigma_theta_ <= 0.0)
            {
                ROS_FATAL("[DEKF] observation noise sigma must be > 0");
                ok = false;
            }
            if (max_observation_delay_ <= current_delay_threshold_)
            {
                ROS_FATAL("[DEKF] max_observation_delay=%.3f must be > "
                          "current_delay_threshold=%.3f",
                          max_observation_delay_, current_delay_threshold_);
                ok = false;
            }
            if (max_nis_dgo_ <= 0.0 || max_nis_uwb_ <= 0.0 ||
                max_nis_camera_1d_ <= 0.0 || max_nis_camera_2d_ <= 0.0)
            {
                ROS_FATAL("[DEKF] per-observation NIS gates must be > 0");
                ok = false;
            }
            if (uwb_residual_gate_ <= 0.0 || dgo_residual_gate_ <= 0.0 ||
                camera_residual_gate_ <= 0.0)
            {
                ROS_FATAL("[DEKF] residual gates must be > 0");
                ok = false;
            }
            if (uwb_anchor_max_age_ <= 0.0 ||
                dgo_recovery_gate_ <= 0.0 ||
                dgo_recovery_count_threshold_ < 1 ||
                dgo_recovery_consistency_gate_ <= 0.0 ||
                dgo_recovery_pos_std_ <= 0.0 ||
                dgo_recovery_vel_std_ <= 0.0)
            {
                ROS_FATAL("[DEKF] recovery / anchor parameters are invalid");
                ok = false;
            }
            if (!ok)
            {
                ros::shutdown();
                return;
            }
        }

        dgo_cache_.resize(uav_num_);

        // 编队初始 offset: 2x2 方阵, z=0 (已知编队)
        initial_offsets_.resize(uav_num_);
        initial_offsets_[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
        initial_offsets_[1] = Eigen::Vector3d(initial_spacing_, 0.0, 0.0);
        initial_offsets_[2] = Eigen::Vector3d(initial_spacing_, initial_spacing_, 0.0);
        initial_offsets_[3] = Eigen::Vector3d(0.0, initial_spacing_, 0.0);

        ROS_INFO("[DEKF] ns=%s uav_id=%d uav_num=%d rate=%.2fHz initial_spacing=%.3f "
                 "offsets: iris_0=(%.2f,%.2f,%.2f) iris_1=(%.2f,%.2f,%.2f) "
                 "iris_2=(%.2f,%.2f,%.2f) iris_3=(%.2f,%.2f,%.2f)",
                 ros::this_node::getNamespace().c_str(), uav_id_, uav_num_, rate_, initial_spacing_,
                 initial_offsets_[0].x(), initial_offsets_[0].y(), initial_offsets_[0].z(),
                 initial_offsets_[1].x(), initial_offsets_[1].y(), initial_offsets_[1].z(),
                 initial_offsets_[2].x(), initial_offsets_[2].y(), initial_offsets_[2].z(),
                 initial_offsets_[3].x(), initial_offsets_[3].y(), initial_offsets_[3].z());
        ROS_INFO("[DEKF] gates: max_nis legacy=%.2f dgo=%.2f uwb=%.2f "
                 "camera_1d=%.2f camera_2d=%.2f residual_gate dgo=%.2fm "
                 "uwb=%.2fm camera=%.2frad",
                 max_nis_, max_nis_dgo_, max_nis_uwb_,
                 max_nis_camera_1d_, max_nis_camera_2d_,
                 dgo_residual_gate_, uwb_residual_gate_, camera_residual_gate_);
        ROS_INFO("[DEKF] recovery/anchor: require_uwb_anchor=%d uwb_anchor_max_age=%.2fs "
                 "enable_dgo_recovery=%d recovery_gate=%.2fm count=%d consistency=%.2fm "
                 "reset_std_pos=%.2fm reset_std_vel=%.2fm clear_history=%d",
                 require_direction_anchor_for_uwb_ ? 1 : 0,
                 uwb_anchor_max_age_,
                 enable_dgo_recovery_ ? 1 : 0,
                 dgo_recovery_gate_,
                 dgo_recovery_count_threshold_,
                 dgo_recovery_consistency_gate_,
                 dgo_recovery_pos_std_,
                 dgo_recovery_vel_std_,
                 clear_history_on_recovery_ ? 1 : 0);

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

        // 初始化滤波器: 每个 target 一个, 跳过本机, 用编队初始 offset 设置初值
        filters_.resize(uav_num_);
        for (int i = 0; i < uav_num_; ++i)
        {
            filters_[i].target_id = i;
            if (i == uav_id_)
                continue;

            Eigen::Vector3d init_rel =
                initial_offsets_[i] - initial_offsets_[uav_id_];
            filters_[i].X.segment<3>(0) = init_rel;
            filters_[i].X.segment<3>(3).setZero();
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

        openCsv();

	        // DEKF 定时器
	        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_),
	                                 &DEKF::dekfCallback, this);
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
        Eigen::MatrixXd H;       // 观测雅可比 (动态维度, 用于延迟补偿)
        Eigen::MatrixXd K;       // 卡尔曼增益 (用于延迟补偿)
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

        size_t publish_count = 0;
        size_t current_updates = 0;
        size_t delayed_updates = 0;
        size_t dgo_updates = 0;
        size_t uwb_updates = 0;
        size_t camera_updates = 0;
        size_t rejects = 0;
        size_t delayed_p_skips = 0;
        size_t future_requeues = 0;
        size_t old_drops = 0;
        size_t recovery_resets = 0;

        ros::Time last_direction_anchor_stamp;
        int dgo_recovery_count = 0;
        bool has_last_recovery_dgo = false;
        Eigen::Vector3d last_recovery_dgo_position = Eigen::Vector3d::Zero();
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
    double initial_spacing_ = 2.0;

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
    double max_history_match_dt_ = 0.06;
    double future_tolerance_ = 0.02;
    double max_nis_ = 25.0;
    double max_nis_dgo_ = 16.3;
    double max_nis_uwb_ = 9.0;
    double max_nis_camera_1d_ = 9.0;
    double max_nis_camera_2d_ = 13.8;
    double uwb_residual_gate_ = 0.8;
    double dgo_residual_gate_ = 2.0;
    double camera_residual_gate_ = 0.5;
    bool require_direction_anchor_for_uwb_ = true;
    double uwb_anchor_max_age_ = 0.50;
    bool enable_dgo_recovery_ = true;
    double dgo_recovery_gate_ = 1.0;
    int dgo_recovery_count_threshold_ = 3;
    double dgo_recovery_consistency_gate_ = 0.50;
    double dgo_recovery_pos_std_ = 0.20;
    double dgo_recovery_vel_std_ = 0.50;
    bool clear_history_on_recovery_ = true;
    int max_pending_ = 100;
    double history_keep_time_ = 2.0;
    bool initialized_ = false;

    // CSV 诊断输出
    std::string csv_dir_;
    std::ofstream csv_;

    // 订阅器 & 数据缓存
    std::vector<ros::Subscriber> dgo_subs_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;
    std::vector<Filter> filters_;
    std::vector<std::deque<DGOSample>> dgo_cache_;

    // 编队初始 offset (用于 DGO 相对位置补齐)
    std::vector<Eigen::Vector3d> initial_offsets_;

    // ═══════════════════════════════════════════════════════════
    //  CSV 诊断输出
    // ═══════════════════════════════════════════════════════════

    static bool ensureDirectory(const std::string &path)
    {
        if (path.empty())
            return false;

        std::string current;
        if (path[0] == '/')
            current = "/";

        size_t start = (path[0] == '/') ? 1 : 0;
        while (start <= path.size())
        {
            const size_t slash = path.find('/', start);
            const std::string part = path.substr(start, slash - start);
            if (!part.empty())
            {
                if (!current.empty() && current[current.size() - 1] != '/')
                    current += "/";
                current += part;

                struct stat st;
                if (stat(current.c_str(), &st) != 0)
                {
                    if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
                        return false;
                }
                else if (!S_ISDIR(st.st_mode))
                {
                    return false;
                }
            }

            if (slash == std::string::npos)
                break;
            start = slash + 1;
        }

        return true;
    }

    const char *obsTypeName(ObsType type) const
    {
        switch (type)
        {
            case ObsType::DGO:
                return "dgo";
            case ObsType::UWB_RANGE:
                return "uwb";
            case ObsType::CAMERA_BEARING:
                return "camera";
        }
        return "unknown";
    }

    void openCsv()
    {
        if (csv_dir_.empty())
            return;

        if (!ensureDirectory(csv_dir_))
        {
            ROS_WARN("[DEKF] failed to create csv_dir=%s", csv_dir_.c_str());
            return;
        }

        const std::string self_name = "iris_" + std::to_string(uav_id_);
        const std::string path = csv_dir_ + "/" + self_name + "_my_dekf_debug.csv";
        csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv_.is_open())
        {
            ROS_WARN("[DEKF] cannot open CSV: %s", path.c_str());
            return;
        }

        csv_ << "time,event,target,obs_type,accepted,delayed,p_updated,reason,"
             << "obs_stamp,filter_stamp,age,history_match_dt,residual_norm,nis,"
             << "x,y,z,vx,vy,vz,"
             << "Pxx,Pyy,Pzz,Pvxvx,Pvyvy,Pvzvz,"
             << "pending_size,cache_size,publish_count,current_updates,delayed_updates,"
             << "dgo_updates,uwb_updates,camera_updates,rejects,delayed_p_skips,"
             << "future_requeues,old_drops,recovery_resets,dgo_recovery_count,"
             << "last_direction_anchor_age\n";

        ROS_INFO("[DEKF] debug CSV: %s", path.c_str());
    }

    void logCsvRow(const std::string &event,
                   const Filter &f,
                   const char *obs_type,
                   int accepted,
                   int delayed,
                   int p_updated,
                   const std::string &reason,
                   double obs_stamp,
                   double age,
                   double history_match_dt,
                   double residual_norm,
                   double nis)
    {
        if (!csv_.is_open())
            return;

        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double t = f.stamp.isZero() ? ros::Time::now().toSec() : f.stamp.toSec();
        const double filter_stamp = f.stamp.isZero() ? nan : f.stamp.toSec();
        const double anchor_age =
            f.last_direction_anchor_stamp.isZero() || f.stamp.isZero()
                ? nan
                : (f.stamp - f.last_direction_anchor_stamp).toSec();

        csv_ << std::fixed << std::setprecision(9)
             << t << ","
             << event << ","
             << f.target_id << ","
             << obs_type << ","
             << accepted << ","
             << delayed << ","
             << p_updated << ","
             << reason << ","
             << obs_stamp << ","
             << filter_stamp << ","
             << age << ","
             << history_match_dt << ","
             << residual_norm << ","
             << nis << ","
             << f.X[0] << "," << f.X[1] << "," << f.X[2] << ","
             << f.X[3] << "," << f.X[4] << "," << f.X[5] << ","
             << f.P(0, 0) << "," << f.P(1, 1) << "," << f.P(2, 2) << ","
             << f.P(3, 3) << "," << f.P(4, 4) << "," << f.P(5, 5) << ","
             << f.pending_obs.size() << ","
             << f.cache.size() << ","
             << f.publish_count << ","
             << f.current_updates << ","
             << f.delayed_updates << ","
             << f.dgo_updates << ","
             << f.uwb_updates << ","
             << f.camera_updates << ","
             << f.rejects << ","
             << f.delayed_p_skips << ","
             << f.future_requeues << ","
             << f.old_drops << ","
             << f.recovery_resets << ","
             << f.dgo_recovery_count << ","
             << anchor_age << "\n";
    }

    void logObservationRow(const std::string &event,
                           const Filter &f,
                           const Observation &obs,
                           int accepted,
                           int delayed,
                           int p_updated,
                           const std::string &reason,
                           double age,
                           double history_match_dt,
                           double residual_norm,
                           double nis)
    {
        const double obs_stamp =
            obs.stamp.isZero() ? std::numeric_limits<double>::quiet_NaN() : obs.stamp.toSec();
        logCsvRow(event, f, obsTypeName(obs.type), accepted, delayed, p_updated,
                  reason, obs_stamp, age, history_match_dt, residual_norm, nis);
    }

    void logPublishRow(const Filter &f)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        logCsvRow("publish", f, "none", 1, 0, 0, "publish",
                  nan, nan, nan, nan, nan);
    }

    bool hasRecentDirectionAnchor(const Filter &f) const
    {
        if (!require_direction_anchor_for_uwb_)
            return true;
        if (f.last_direction_anchor_stamp.isZero() || f.stamp.isZero())
            return false;
        const double age = (f.stamp - f.last_direction_anchor_stamp).toSec();
        return std::isfinite(age) && age >= 0.0 && age <= uwb_anchor_max_age_;
    }

    void markDirectionAnchor(Filter &f)
    {
        f.last_direction_anchor_stamp = f.stamp;
        f.dgo_recovery_count = 0;
        f.has_last_recovery_dgo = false;
    }

    void rebuildHistoryAfterRecovery(Filter &f)
    {
        f.cache.clear();

        History_Record rec;
        rec.stamp = f.stamp;
        rec.X_pred = f.X;
        rec.P_pred = f.P;
        rec.A.setIdentity();
        rec.I_KH.setIdentity();
        rec.H.resize(0, 0);
        rec.K.resize(0, 0);
        f.cache.push_back(rec);
    }

    void resetFilterToDgo(Filter &f,
                          const Observation &obs,
                          bool delayed,
                          double age,
                          double history_match_dt,
                          double residual_norm,
                          double nis,
                          const std::string &reason)
    {
        const double dt_comp =
            std::max(0.0, std::min(age, max_observation_delay_));

        Eigen::Vector3d recovered_pos = obs.position;
        Eigen::Vector3d recovered_vel = Eigen::Vector3d::Zero();
        if (obs.velocity.allFinite())
        {
            recovered_pos += obs.velocity * dt_comp;
            recovered_vel = obs.velocity;
        }

        f.X.segment<3>(0) = recovered_pos;
        f.X.segment<3>(3) = recovered_vel;
        f.P.setZero();
        f.P.block<3, 3>(0, 0).diagonal().setConstant(
            dgo_recovery_pos_std_ * dgo_recovery_pos_std_);
        f.P.block<3, 3>(3, 3).diagonal().setConstant(
            dgo_recovery_vel_std_ * dgo_recovery_vel_std_);
        symmetrizeCov(f.P);

        ++f.recovery_resets;
        incrementAcceptedCounters(f, obs, delayed);
        markDirectionAnchor(f);

        if (clear_history_on_recovery_)
            rebuildHistoryAfterRecovery(f);

        ROS_WARN("[DEKF] recovery reset target=%d delayed=%d reason=%s "
                 "age=%.3f residual=%.3f pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f)",
                 f.target_id,
                 delayed ? 1 : 0,
                 reason.c_str(),
                 age,
                 residual_norm,
                 f.X[0], f.X[1], f.X[2],
                 f.X[3], f.X[4], f.X[5]);

        logObservationRow(delayed ? "delayed_update" : "current_update",
                          f, obs, 1, delayed ? 1 : 0, 1,
                          reason, age, history_match_dt,
                          residual_norm, nis);
    }

    bool tryDgoRecovery(Filter &f,
                        const Observation &obs,
                        bool delayed,
                        double age,
                        double history_match_dt,
                        double residual_norm,
                        double nis,
                        const std::string &source_reason)
    {
        if (!enable_dgo_recovery_ || obs.type != ObsType::DGO ||
            !obs.position.allFinite() || residual_norm < dgo_recovery_gate_)
        {
            return false;
        }

        bool consistent = true;
        if (f.has_last_recovery_dgo)
        {
            consistent =
                (obs.position - f.last_recovery_dgo_position).norm() <=
                dgo_recovery_consistency_gate_;
        }

        f.last_recovery_dgo_position = obs.position;
        f.has_last_recovery_dgo = true;
        f.dgo_recovery_count = consistent ? f.dgo_recovery_count + 1 : 1;

        if (f.dgo_recovery_count >= dgo_recovery_count_threshold_)
        {
            resetFilterToDgo(f, obs, delayed, age, history_match_dt,
                             residual_norm, nis,
                             "dgo_recovery_reset_after_" + source_reason);
        }
        else
        {
            ++f.rejects;
            logObservationRow(delayed ? "delayed_update" : "current_update",
                              f, obs, 0, delayed ? 1 : 0, 0,
                              "dgo_recovery_pending_after_" + source_reason,
                              age, history_match_dt, residual_norm, nis);
        }

        return true;
    }

    void incrementAcceptedCounters(Filter &f, const Observation &obs, bool delayed)
    {
        if (delayed)
            ++f.delayed_updates;
        else
            ++f.current_updates;

        switch (obs.type)
        {
            case ObsType::DGO:
                ++f.dgo_updates;
                markDirectionAnchor(f);
                break;
            case ObsType::UWB_RANGE:
                ++f.uwb_updates;
                break;
            case ObsType::CAMERA_BEARING:
                ++f.camera_updates;
                markDirectionAnchor(f);
                break;
        }
    }

    double nisGateForObservation(const Observation &obs, int residual_dim) const
    {
        switch (obs.type)
        {
            case ObsType::DGO:
                return max_nis_dgo_;
            case ObsType::UWB_RANGE:
                return max_nis_uwb_;
            case ObsType::CAMERA_BEARING:
                return residual_dim <= 1 ? max_nis_camera_1d_ : max_nis_camera_2d_;
        }
        return max_nis_;
    }

    bool passHardResidualGate(const Observation &obs,
                              const Eigen::VectorXd &residual,
                              std::string &reason) const
    {
        if (!residual.allFinite())
        {
            reason = "nonfinite_residual";
            return false;
        }

        switch (obs.type)
        {
            case ObsType::DGO:
                if (residual.norm() > dgo_residual_gate_)
                {
                    reason = "dgo_residual_gate_reject";
                    return false;
                }
                return true;

            case ObsType::UWB_RANGE:
                if (residual.size() < 1 ||
                    std::abs(residual(0)) > uwb_residual_gate_)
                {
                    reason = "uwb_residual_gate_reject";
                    return false;
                }
                return true;

            case ObsType::CAMERA_BEARING:
                if (residual.size() < 1 ||
                    residual.cwiseAbs().maxCoeff() > camera_residual_gate_)
                {
                    reason = "camera_residual_gate_reject";
                    return false;
                }
                return true;
        }

        reason = "unknown_observation_type";
        return false;
    }

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
            obs.position.x() = (initial_offsets_[target].x() - initial_offsets_[self_id].x())
                             + peer.msg.pose.pose.position.x - self.msg.pose.pose.position.x;
            obs.position.y() = (initial_offsets_[target].y() - initial_offsets_[self_id].y())
                             + peer.msg.pose.pose.position.y - self.msg.pose.pose.position.y;
            obs.position.z() = (initial_offsets_[target].z() - initial_offsets_[self_id].z())
                             + peer.msg.pose.pose.position.z - self.msg.pose.pose.position.z;
            obs.velocity.x() = peer.msg.twist.twist.linear.x - self.msg.twist.twist.linear.x;
            obs.velocity.y() = peer.msg.twist.twist.linear.y - self.msg.twist.twist.linear.y;
            obs.velocity.z() = peer.msg.twist.twist.linear.z - self.msg.twist.twist.linear.z;

            if (!obs.position.allFinite() || !obs.velocity.allFinite())
                continue;

            filters_[target].pending_obs.push_back(obs);
            while (filters_[target].pending_obs.size() > static_cast<size_t>(max_pending_))
                filters_[target].pending_obs.pop_front();
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
            while (filters_[target].pending_obs.size() > static_cast<size_t>(max_pending_))
                filters_[target].pending_obs.pop_front();
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
            while (filters_[target].pending_obs.size() > static_cast<size_t>(max_pending_))
                filters_[target].pending_obs.pop_front();
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

        // use_sim_time 下, 仿真时钟尚未就绪时 now 为 0
        if (now.isZero())
            return;

        // 首次触发时初始化所有滤波器时间戳
        if (!initialized_)
        {
            for (int target = 0; target < uav_num_; ++target)
            {
                if (target == uav_id_)
                    continue;
                Filter &f = filters_[target];
                f.stamp = now;
            }
            initialized_ = true;
        }

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

                if (age < -future_tolerance_)
                {
                    // 观测时间戳在未来 → 跳过, 等后续周期再处理
                    f.pending_obs.push_back(obs);
                    while (f.pending_obs.size() > static_cast<size_t>(max_pending_))
                        f.pending_obs.pop_front();
                    ++f.future_requeues;
                    logObservationRow("requeue_future", f, obs, 0, 0, 0,
                                      "future_observation", age,
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN());
                }
                else if (age < current_delay_threshold_)
                {
                    // 观测时间 ≈ 当前时间 → 直接在当前状态上更新
                    applyCurrentUpdate(f, obs);
                }
                else if (age < max_observation_delay_)
                {
                    applyDelayedUpdate(f, obs);
                }
                else
                {
                    // 观测太旧, 直接丢弃
                    ++f.old_drops;
                    ++f.rejects;
                    logObservationRow("drop_old", f, obs, 0, 0, 0,
                                      "observation_too_old", age,
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN());
                }
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

        // 用编队初始 offset 作为相对位姿初值, 而非全零
        // 这样 UWB/Camera 的初始线性化更准确
        Eigen::Vector3d init_rel = Eigen::Vector3d::Zero();
        if (f.target_id >= 0 && f.target_id < uav_num_)
            init_rel = initial_offsets_[f.target_id] - initial_offsets_[uav_id_];
        f.X.segment<3>(0) = init_rel;
        f.X.segment<3>(3).setZero();
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

        // 初始化延迟补偿缓存 (无更新时 I_KH=Identity, H/K 为空)
        rec.I_KH.setIdentity();
        rec.H.resize(0, 0);
        rec.K.resize(0, 0);

        f.X = rec.X_pred;
        f.P = rec.P_pred;
        symmetrizeCov(f.P);
        f.stamp = stamp;

        f.cache.push_back(rec);
        CleanCacheOverflow(f);
    }

    // 清理过期的预测缓存, 按时间保留 (默认保留最近 2s)
    void CleanCacheOverflow(Filter &f)
    {
        const ros::Time now = f.stamp;
        while (f.cache.size() > 2 &&
               (now - f.cache.front().stamp).toSec() > history_keep_time_)
        {
            f.cache.pop_front();
        }
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
                double px = x(0), py = x(1), pz = x(2);
                double r = sqrt(px*px + py*py + pz*pz);
                if (r < 1e-6)
                    return false;

                Z.resize(1);
                residual.resize(1);
                Z << obs.value;
                residual(0) = Z(0) - r;
                H.resize(1, 6);
                H << px/r, py/r, pz/r, 0.0, 0.0, 0.0;
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
                if (r < 1e-6 || l < 1e-6)
                    return false;

                double alpha_pred = std::atan2(py, px);
                double ratio = std::max(-1.0, std::min(1.0, pz / r));
                double theta_pred = std::asin(ratio);

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
    //  协方差维护
    // ═══════════════════════════════════════════════════════════

    void symmetrizeCov(Matrix6d &P) const
    {
        P = 0.5 * (P + P.transpose());
        for (int i = 0; i < 6; ++i)
        {
            if (!std::isfinite(P(i, i)) || P(i, i) < 1e-9)
                P(i, i) = 1e-9;
        }
    }

    bool isUsableCovariance(const Matrix6d &P) const
    {
        if (!P.allFinite())
            return false;

        Matrix6d P_sym = 0.5 * (P + P.transpose());
        for (int i = 0; i < 6; ++i)
        {
            if (!std::isfinite(P_sym(i, i)) || P_sym(i, i) <= 1e-9)
                return false;
        }

        Eigen::LLT<Matrix6d> llt(P_sym);
        return llt.info() == Eigen::Success;
    }

    // ═══════════════════════════════════════════════════════════
    //  观测更新函数
    // ═══════════════════════════════════════════════════════════

    // 当前时刻 EKF 更新: 观测时间 ≈ 当前预测时间, 直接执行标准 EKF 更新
    void applyCurrentUpdate(Filter &f, const Observation &obs)
    {
        const double age = (f.stamp - obs.stamp).toSec();
        Eigen::VectorXd Z;
        Eigen::VectorXd residual;
        Eigen::MatrixXd H;
        Eigen::MatrixXd R_meas;

        if (!buildObsModel(f.X, obs, Z, residual, H, R_meas))
        {
            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              "build_obs_model_failed", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        const double residual_norm = residual.norm();
        if (obs.type == ObsType::UWB_RANGE && !hasRecentDirectionAnchor(f))
        {
            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              "uwb_no_recent_direction_anchor", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        std::string gate_reason;
        if (!passHardResidualGate(obs, residual, gate_reason))
        {
            if (tryDgoRecovery(f, obs, false, age,
                               std::numeric_limits<double>::quiet_NaN(),
                               residual_norm,
                               std::numeric_limits<double>::quiet_NaN(),
                               gate_reason))
            {
                return;
            }

            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              gate_reason, age,
                              std::numeric_limits<double>::quiet_NaN(),
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        // ── NIS gate: 拒绝异常观测 ──────────────────────────
        Eigen::MatrixXd S = H * f.P * H.transpose() + R_meas;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
        if (!lu.isInvertible())
        {
            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              "innovation_cov_not_invertible", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }
        const Eigen::MatrixXd S_inv = lu.inverse();
        const double nis = residual.transpose() * S_inv * residual;
        const double nis_gate = nisGateForObservation(obs, residual.size());
        if (nis > nis_gate)
        {
            if (tryDgoRecovery(f, obs, false, age,
                               std::numeric_limits<double>::quiet_NaN(),
                               residual_norm, nis, "nis_reject"))
            {
                return;
            }

            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              "nis_reject", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              residual_norm,
                              nis);
            return;
        }

        // 标准卡尔曼增益: K = P * H^T * (H * P * H^T + R)^{-1}
        Eigen::MatrixXd K = f.P * H.transpose() * S_inv;
        f.X += K * residual;

        // Joseph 形式协方差更新 (数值稳定性优于标准形式)
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
        f.P = I_KH * f.P * I_KH.transpose() + K * R_meas * K.transpose();
        symmetrizeCov(f.P);
        incrementAcceptedCounters(f, obs, false);

        // 缓存 H, K, I_KH 到最新的历史记录 (用于延迟补偿重传播)
        if (!f.cache.empty())
        {
            f.cache.back().H = H;
            f.cache.back().K = K;
            // 累乘: 同一历史时刻多次 update 的 I_KH 组合
            f.cache.back().I_KH = I_KH * f.cache.back().I_KH;
        }

        logObservationRow("current_update", f, obs, 1, 0, 1,
                          "accepted", age,
                          std::numeric_limits<double>::quiet_NaN(),
                          residual_norm,
                          nis);
    }

    void applyDelayedUpdate(Filter &f, const Observation &obs)
    {
        const double age = (f.stamp - obs.stamp).toSec();
        if (f.cache.empty())
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "empty_history_cache", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        // 从前往后找第一个 cache[i].stamp >= obs.stamp 的记录
        int best_idx = -1;
        double best_dt = std::numeric_limits<double>::max();
        for (size_t i = 0; i < f.cache.size(); ++i)
        {
            double dt = (f.cache[i].stamp - obs.stamp).toSec();
            if (dt >= 0.0)
            {
                best_idx = static_cast<int>(i);
                best_dt = dt;
                break;
            }
        }
        if (best_idx < 0 || best_dt > max_history_match_dt_)
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "history_match_failed", age,
                              best_dt,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        const History_Record &hist = f.cache[best_idx];

        Eigen::VectorXd Zs;
        Eigen::VectorXd residual;
        Eigen::MatrixXd Hs;
        Eigen::MatrixXd R_meas;

        if (!buildObsModel(hist.X_pred, obs, Zs, residual, Hs, R_meas))
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "build_obs_model_failed", age,
                              best_dt,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        // 在历史状态处计算 Kalman 增益
        const double residual_norm = residual.norm();
        if (obs.type == ObsType::UWB_RANGE && !hasRecentDirectionAnchor(f))
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "uwb_no_recent_direction_anchor", age,
                              best_dt,
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        std::string gate_reason;
        if (!passHardResidualGate(obs, residual, gate_reason))
        {
            if (tryDgoRecovery(f, obs, true, age, best_dt,
                               residual_norm,
                               std::numeric_limits<double>::quiet_NaN(),
                               gate_reason))
            {
                return;
            }

            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              gate_reason, age,
                              best_dt,
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        Eigen::MatrixXd S = Hs * hist.P_pred * Hs.transpose() + R_meas;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
        if (!lu.isInvertible())
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "innovation_cov_not_invertible", age,
                              best_dt,
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN());
            return;
        }

        const Eigen::MatrixXd S_inv = lu.inverse();
        const double nis = residual.transpose() * S_inv * residual;
        const double nis_gate = nisGateForObservation(obs, residual.size());
        if (nis > nis_gate)
        {
            if (tryDgoRecovery(f, obs, true, age, best_dt,
                               residual_norm, nis, "nis_reject"))
            {
                return;
            }

            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "nis_reject", age,
                              best_dt,
                              residual_norm,
                              nis);
            return;
        }
        Eigen::MatrixXd K_s = hist.P_pred * Hs.transpose() * S_inv;

        // 修正量 = K_s * residual, 并用同一条 A/I_KH 链传播 K_s 到当前时刻。
        // Wk 维度为 6 x m, m 是当前观测维度(DGO=3, UWB=1, Camera=1/2)。
        Eigen::MatrixXd Wk = K_s;
        for (size_t i = static_cast<size_t>(best_idx) + 1; i < f.cache.size(); ++i)
        {
            Wk = f.cache[i].A * Wk;
            Wk = f.cache[i].I_KH * Wk;
        }

        Vector6d delta = Wk * residual;
        if (!delta.allFinite() || !Wk.allFinite())
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "nonfinite_delta_or_Wk", age,
                              best_dt,
                              residual_norm,
                              nis);
            return;
        }

        // 原子化 delayed update:
        // 先同时计算 X_new/P_new, 只有协方差合法时才提交状态和协方差。
        const Vector6d X_new = f.X + delta;

        // 近似 delayed covariance update:
        // P_k = P_k - W_k * S_s * W_k^T.
        Matrix6d P_new = f.P - (Wk * S * Wk.transpose());
        if (!X_new.allFinite() || !isUsableCovariance(P_new))
        {
            ++f.delayed_p_skips;
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "atomic_update_rejected", age,
                              best_dt,
                              residual_norm,
                              nis);
            return;
        }

        f.X = X_new;
        f.P = P_new;
        symmetrizeCov(f.P);
        incrementAcceptedCounters(f, obs, true);

        logObservationRow("delayed_update", f, obs, 1, 1, 1,
                          "accepted", age,
                          best_dt,
                          residual_norm,
                          nis);
    }

    // ═══════════════════════════════════════════════════════════
    //  发布函数
    // ═══════════════════════════════════════════════════════════

    // 发布 DEKF 估计结果: /iris_{self}/dekf/iris_{target}
    void publishFilter(Filter &f)
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
        ++f.publish_count;
        logPublishRow(f);
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
