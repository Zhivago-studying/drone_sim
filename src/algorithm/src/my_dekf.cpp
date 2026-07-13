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
#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
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
        pnh_.param("sigma_px", sigma_px_, 0.08);
        pnh_.param("sigma_py", sigma_py_, 0.08);
        pnh_.param("sigma_pz", sigma_pz_, 0.08);
        pnh_.param("sigma_uwb", sigma_uwb_, 0.05);
        pnh_.param("sigma_alpha", sigma_alpha_, 0.05);
        pnh_.param("sigma_theta", sigma_theta_, 0.05);
        pnh_.param("use_dgo_velocity", use_dgo_velocity_, false);
        pnh_.param("dgo_velocity_noise_std", dgo_velocity_noise_std_, 0.25);
        pnh_.param("max_dgo_pair_dt", max_dgo_pair_dt_, 0.12);
        pnh_.param("max_observation_delay", max_observation_delay_, 0.60);
        pnh_.param("current_delay_threshold", current_delay_threshold_, 0.020);
        pnh_.param("max_history_match_dt", max_history_match_dt_, 0.06);
        pnh_.param("future_tolerance", future_tolerance_, 0.02);
        pnh_.param("delayed_update_gain_weight", delayed_update_gain_weight_, 1.0);
        if (!pnh_.hasParam("delayed_update_gain_weight"))
        {
            double legacy_delayed_covariance_weight = delayed_update_gain_weight_;
            if (pnh_.getParam("delayed_covariance_weight", legacy_delayed_covariance_weight))
            {
                delayed_update_gain_weight_ = legacy_delayed_covariance_weight;
                ROS_WARN("[DEKF] parameter delayed_covariance_weight is deprecated; "
                         "use delayed_update_gain_weight. Interpreting value as gain damping.");
            }
        }
        pnh_.param("max_nis", max_nis_, 25.0);
        pnh_.param("max_nis_dgo", max_nis_dgo_, 16.3);
        pnh_.param("max_nis_uwb", max_nis_uwb_, 12.0);
        pnh_.param("max_nis_camera_1d", max_nis_camera_1d_, 9.0);
        pnh_.param("max_nis_camera_2d", max_nis_camera_2d_, 13.8);
        pnh_.param("uwb_residual_gate", uwb_residual_gate_, 0.8);
        pnh_.param("dgo_residual_gate", dgo_residual_gate_, 2.0);
        pnh_.param("dgo_velocity_residual_gate", dgo_velocity_residual_gate_, 2.0);
        pnh_.param("camera_residual_gate", camera_residual_gate_, 0.5);
        pnh_.param("require_direction_anchor_for_uwb", require_direction_anchor_for_uwb_, true);
	        pnh_.param("uwb_anchor_max_age", uwb_anchor_max_age_, 0.50);
	        pnh_.param("delayed_validation_enabled", delayed_validation_enabled_, false);
	        pnh_.param("delayed_validation_gt_max_dt", delayed_validation_gt_max_dt_, 0.05);
	        pnh_.param("mode", fusion_mode_, 4);
	        pnh_.param("trace_enabled", trace_enabled_, false);
	        pnh_.param("trace_dgo_only", trace_dgo_only_, true);
        ROS_DEBUG("[DEKF] param mode=%d", fusion_mode_);

        // 根据 fusion_mode 设置传感器开关
        enable_dgo_update_ = true;
        switch (fusion_mode_)
        {
            case 1:
                enable_uwb_update_ = false;
                enable_camera_update_ = false;
                break;
            case 2:
                enable_uwb_update_ = true;
                enable_camera_update_ = false;
                break;
            case 3:
                enable_uwb_update_ = false;
                enable_camera_update_ = true;
                break;
            case 4:
                enable_uwb_update_ = true;
                enable_camera_update_ = true;
                break;
            default:
                ROS_WARN("[DEKF] invalid fusion_mode=%d, fallback to mode=4 (DGO-UWB-Cam)", fusion_mode_);
                fusion_mode_ = 4;
                enable_uwb_update_ = true;
                enable_camera_update_ = true;
                break;
        }
        ROS_INFO("[DEKF] fusion mode=%d: DGO=%d UWB=%d Camera=%d",
                 fusion_mode_,
                 enable_dgo_update_ ? 1 : 0,
                 enable_uwb_update_ ? 1 : 0,
                 enable_camera_update_ ? 1 : 0);
        pnh_.param("enable_dgo_recovery", enable_dgo_recovery_, true);
        pnh_.param("dgo_recovery_gate", dgo_recovery_gate_, 3.0);
        pnh_.param("dgo_recovery_count_threshold", dgo_recovery_count_threshold_, 3);
        pnh_.param("dgo_recovery_consistency_gate", dgo_recovery_consistency_gate_, 0.50);
        pnh_.param("dgo_recovery_cooldown", dgo_recovery_cooldown_, 1.0);
        pnh_.param("dgo_recovery_pos_std", dgo_recovery_pos_std_, 0.20);
        pnh_.param("dgo_recovery_vel_std", dgo_recovery_vel_std_, 0.50);
        pnh_.param("clear_history_on_recovery", clear_history_on_recovery_, true);
        pnh_.param("allow_dgo_x_only_on_bad_cov", allow_dgo_x_only_on_bad_cov_, false);
        pnh_.param("max_position_cov", max_position_cov_, 2.0);
        pnh_.param("max_velocity_cov", max_velocity_cov_, 1.0);
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
            if (delayed_update_gain_weight_ <= 0.0 || delayed_update_gain_weight_ > 1.0)
            {
                ROS_FATAL("[DEKF] delayed_update_gain_weight=%.3f must be in (0, 1]",
                          delayed_update_gain_weight_);
                ok = false;
            }
            if (max_nis_dgo_ <= 0.0 || max_nis_uwb_ <= 0.0 ||
                max_nis_camera_1d_ <= 0.0 || max_nis_camera_2d_ <= 0.0)
            {
                ROS_FATAL("[DEKF] per-observation NIS gates must be > 0");
                ok = false;
            }
            if (uwb_residual_gate_ <= 0.0 || dgo_residual_gate_ <= 0.0 ||
                dgo_velocity_residual_gate_ <= 0.0 ||
                camera_residual_gate_ <= 0.0)
            {
                ROS_FATAL("[DEKF] residual gates must be > 0");
                ok = false;
            }
            if (uwb_anchor_max_age_ <= 0.0 ||
                dgo_recovery_gate_ <= 0.0 ||
                dgo_recovery_count_threshold_ < 1 ||
                dgo_recovery_consistency_gate_ <= 0.0 ||
                dgo_recovery_cooldown_ < 0.0 ||
                dgo_recovery_pos_std_ <= 0.0 ||
                dgo_recovery_vel_std_ <= 0.0)
            {
                ROS_FATAL("[DEKF] recovery / anchor parameters are invalid");
                ok = false;
            }
            if (max_position_cov_ <= 0.0 || max_velocity_cov_ <= 0.0)
            {
                ROS_FATAL("[DEKF] covariance bounds must be > 0: pos=%.3f vel=%.3f",
                          max_position_cov_, max_velocity_cov_);
                ok = false;
            }
            if (!ok)
            {
                ros::shutdown();
                return;
            }
        }

        // use_dgo_velocity 参数合法性检查
        if (use_dgo_velocity_ && dgo_velocity_noise_std_ <= 0.0)
        {
            ROS_FATAL("[DEKF] dgo_velocity_noise_std must be positive when use_dgo_velocity is true");
            ros::shutdown();
            return;
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
                 "uwb=%.2fm camera=%.2frad delayed_replay_gain_weight=%.3f",
                 max_nis_, max_nis_dgo_, max_nis_uwb_,
                 max_nis_camera_1d_, max_nis_camera_2d_,
                 dgo_residual_gate_, uwb_residual_gate_, camera_residual_gate_,
                 delayed_update_gain_weight_);
        ROS_INFO("[DEKF] recovery/anchor: require_uwb_anchor=%d uwb_anchor_max_age=%.2fs "
                 "enable_dgo_recovery=%d recovery_gate=%.2fm count=%d consistency=%.2fm cooldown=%.2fs "
                 "reset_std_pos=%.2fm reset_std_vel=%.2fm clear_history=%d "
                 "allow_dgo_x_only_on_bad_cov=%d",
                 require_direction_anchor_for_uwb_ ? 1 : 0,
                 uwb_anchor_max_age_,
                 enable_dgo_recovery_ ? 1 : 0,
                 dgo_recovery_gate_,
                 dgo_recovery_count_threshold_,
                 dgo_recovery_consistency_gate_,
                 dgo_recovery_cooldown_,
                 dgo_recovery_pos_std_,
                 dgo_recovery_vel_std_,
                 clear_history_on_recovery_ ? 1 : 0,
                 allow_dgo_x_only_on_bad_cov_ ? 1 : 0);
        ROS_INFO("[DEKF] covariance bounds: max_position_cov=%.3f max_velocity_cov=%.3f",
                 max_position_cov_, max_velocity_cov_);

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

		    // GT 真值订阅 (用于 delayed update 验证 + trace, 从 /gazebo/model_states 获取)
			    if (delayed_validation_enabled_ || trace_enabled_)
			    {
			        model_gt_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
			            "/gazebo/model_states", 100,
			            &DEKF::gtModelStatesCb, this);
			    }

		    // 任务阶段订阅
		    stage_sub_ = nh_.subscribe<std_msgs::UInt8>(
		        "/formation/stage", 1, &DEKF::stageCallback, this);

		    openCsv();

	        // DEKF 定时器
	        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_),
		                                 &DEKF::dekfCallback, this);
		    }

		    ~DEKF()
		    {
		        writeDelayedAgeSummary();
		        if (delayed_trace_csv_.is_open()) delayed_trace_csv_.close();
		        if (replay_trace_csv_.is_open()) replay_trace_csv_.close();
		        if (state_trace_csv_.is_open()) state_trace_csv_.close();
		    }

	private:
    // ═══════════════════════════════════════════════════════════
    //  类型定义
    // ═══════════════════════════════════════════════════════════

    // 观测类型枚举
    enum class ObsType
    {
        DGO,             // DGO 相对位姿观测
        UWB_RANGE,       // UWB 测距观测
        CAMERA_BEARING   // 相机角度观测
    };

	    // GT 真值采样 (来自 /gazebo/model_states, 用于 delayed update 验证)
	struct GtWorldSample
	{
	    ros::Time stamp;
	    std::array<Eigen::Vector3d, 4> p = {
	        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
	        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
	    };
	    std::array<Eigen::Vector3d, 4> v = {
	        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
	        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
	    };
	};

	    enum class UpdateStatus
    {
        ACCEPTED,
        REJECTED,
        RECOVERY_RESET
    };

    enum class RecoveryResult
    {
        NONE,
        HANDLED_REJECTED,
        RESET
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

    struct AdaptiveUwbInfo
    {
        AdaptiveUwbInfo()
            : noise(std::numeric_limits<double>::quiet_NaN()),
              scale(std::numeric_limits<double>::quiet_NaN()),
              anchor_age(std::numeric_limits<double>::quiet_NaN()),
              reason("none")
        {
        }

        double noise;
        double scale;
        double anchor_age;
        std::string reason;
    };

    // 已经被滤波器接受、需要随 cache replay 重放的观测。
    // 注意：H/K/I_KH 是旧线性化结果，不能替代原始 Observation。
	    struct AppliedObservation
	    {
	        Observation obs;
	        uint64_t seq = 0;          // 同一 cache 节点内保持确定性顺序
	        bool delayed = false;      // 仅用于诊断/调试
	        bool validation_target = false;  // replay 时标记为待验证的 delayed obs
	    };

    // 预测历史缓存 (用于延迟补偿重传播)
    // 语义约定：
    //   X_prior/P_prior: 该 cache 时间戳下，应用本节点 updates 之前的状态。
    //   X_post/P_post:   从 prior 顺序重放 updates 之后的状态。
    // 下一节点预测必须从上一节点 X_post/P_post 出发；延迟 replay 必须从
    // best_idx 节点开始，以确保新插入的 delayed obs 真正作用到该节点。
    struct History_Record
    {
        ros::Time stamp;         // 记录时间戳
        Vector6d X_prior;        // X_{i|i-1} 或该节点 replay 前状态
        Matrix6d P_prior;        // P_{i|i-1} 或该节点 replay 前协方差
        Vector6d X_post;         // X_{i|i}，重放本节点 updates 后状态
        Matrix6d P_post;         // P_{i|i}，重放本节点 updates 后协方差
        Matrix6d A;              // 状态转移矩阵
        Matrix6d I_KH;           // I - K*H  (更新后的投影矩阵)
        Eigen::MatrixXd H;       // 观测雅可比 (动态维度, 用于延迟补偿)
        Eigen::MatrixXd K;       // 卡尔曼增益 (用于延迟补偿)
        std::vector<AppliedObservation> updates; // 该 cache 节点已接受的原始观测
	    };

	    // 延迟更新验证记录 (delayed_validation_enabled_ 时使用)
	    struct DelayedUpdateValidation
	    {
	        bool enabled = false;
	        bool accepted = false;
	        bool replay_ok = false;
	        bool found_target_update = false;

	        int self_id = -1;
	        int target_id = -1;
	        int best_idx = -1;
	        int cache_size = 0;

	        std::string obs_type;
	        std::string reason;

	        ros::Time obs_stamp;
	        ros::Time delayed_update_stamp;
	        ros::Time current_replay_stamp;

	        Vector6d delayed_update_before = Vector6d::Zero();
	        Vector6d delayed_update_after  = Vector6d::Zero();

	        Vector6d current_replay_before = Vector6d::Zero();
	        Vector6d current_replay_after  = Vector6d::Zero();

	        Eigen::Vector3d delayed_update_gt = Eigen::Vector3d::Zero();
	        Eigen::Vector3d current_replay_gt = Eigen::Vector3d::Zero();

	        bool has_delayed_update_gt = false;
	        bool has_current_replay_gt = false;

	        double residual_norm = std::numeric_limits<double>::quiet_NaN();
	        double nis = std::numeric_limits<double>::quiet_NaN();
	        double gain_weight = std::numeric_limits<double>::quiet_NaN();
	    };

		    // ReplayTraceContext — 在 replayCacheFrom 内部收集 delayed update 信息,
		    // 供 applyDelayedUpdate 写 replay trace 使用
		    struct ReplayTraceContext
		    {
		        bool enabled = false;
		        bool found_target_update = false;

		        int self_id = -1;
		        int target_id = -1;
		        int stage = -1;

		        ros::Time obs_stamp;
		        ros::Time hist_stamp;
		        ros::Time current_stamp;

		        int best_idx = -1;
		        int cache_size = 0;

		        Vector6d hist_before = Vector6d::Zero();
		        Vector6d hist_after  = Vector6d::Zero();
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
        ros::Time last_recovery_reset_stamp;
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
    double sigma_px_ = 0.08;
    double sigma_py_ = 0.08;
    double sigma_pz_ = 0.08;
    double sigma_uwb_ = 0.05;
    double sigma_alpha_ = 0.05;
    double sigma_theta_ = 0.05;
    bool use_dgo_velocity_ = false;
    double dgo_velocity_noise_std_ = 0.25;
    Matrix6d R_;

    // 延迟补偿参数
    double max_dgo_pair_dt_ = 0.12;
    double max_observation_delay_ = 0.60;
    double current_delay_threshold_ = 0.020;
    double max_history_match_dt_ = 0.06;
    double future_tolerance_ = 0.02;
    double delayed_update_gain_weight_ = 1.0;
    double max_nis_ = 25.0;
    double max_nis_dgo_ = 16.3;
    double max_nis_uwb_ = 12.0;
    double max_nis_camera_1d_ = 9.0;
    double max_nis_camera_2d_ = 13.8;
    double uwb_residual_gate_ = 0.8;
    double dgo_residual_gate_ = 2.0;
    double dgo_velocity_residual_gate_ = 2.0;
    double camera_residual_gate_ = 0.5;
    bool require_direction_anchor_for_uwb_ = true;
    int fusion_mode_ = 4;
    bool enable_dgo_update_ = true;
    bool enable_uwb_update_ = true;
    bool enable_camera_update_ = true;
	    double uwb_anchor_max_age_ = 0.50;
	    bool delayed_validation_enabled_ = false;
	    double delayed_validation_gt_max_dt_ = 0.05;
	    bool enable_dgo_recovery_ = true;
    double dgo_recovery_gate_ = 3.0;
    int dgo_recovery_count_threshold_ = 3;
    double dgo_recovery_consistency_gate_ = 0.50;
    double dgo_recovery_cooldown_ = 1.0;
    double dgo_recovery_pos_std_ = 0.20;
    double dgo_recovery_vel_std_ = 0.50;
    bool clear_history_on_recovery_ = true;
    bool allow_dgo_x_only_on_bad_cov_ = false;
    double max_position_cov_ = 2.0;
    double max_velocity_cov_ = 1.0;
    int max_pending_ = 100;
    double history_keep_time_ = 2.0;
    bool initialized_ = false;
    uint64_t obs_seq_ = 0;

	    // CSV 诊断输出
	    std::string csv_dir_;
	    std::ofstream csv_;
	    std::ofstream validation_csv_;

	    // Trace 模块 (详细日志, 默认关闭)
	    bool trace_enabled_ = false;
	    bool trace_dgo_only_ = true;
	    std::ofstream delayed_trace_csv_;
	    std::ofstream replay_trace_csv_;
	    std::ofstream state_trace_csv_;

	    // 任务阶段
	    int mission_stage_ = -1;

	    // 订阅器 & 数据缓存
	    std::vector<ros::Subscriber> dgo_subs_;
	    ros::Subscriber uwb_sub_;
	    ros::Subscriber camera_sub_;
	    ros::Subscriber stage_sub_;
	    std::vector<Filter> filters_;
	    std::vector<std::deque<DGOSample>> dgo_cache_;

	    // GT 真值缓存 (来自 /gazebo/model_states, 用于 delayed update 验证)
	    std::deque<GtWorldSample> model_gt_buffer_;
	    ros::Subscriber model_gt_sub_;
	    // 延迟观测 age 统计 (写文件用)
	    double delayed_attempt_age_sum_ = 0.0;
	    size_t delayed_attempt_age_count_ = 0;
	    double delayed_accepted_age_sum_ = 0.0;
	    size_t delayed_accepted_age_count_ = 0;

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
             << "gain_consistency_error,"
             << "uwb_noise_eff,uwb_noise_scale,uwb_anchor_age,uwb_adaptive_reason,"
             << "x,y,z,vx,vy,vz,"
             << "Pxx,Pyy,Pzz,Pvxvx,Pvyvy,Pvzvz,"
             << "pending_size,cache_size,publish_count,current_updates,delayed_updates,"
             << "dgo_updates,uwb_updates,camera_updates,rejects,delayed_p_skips,"
             << "future_requeues,old_drops,recovery_resets,dgo_recovery_count,"
             << "last_direction_anchor_age,last_recovery_reset_age,"
             << "fusion_mode,enable_dgo,enable_uwb,enable_camera\n";

	        ROS_INFO("[DEKF] debug CSV: %s", path.c_str());

	        // 延迟更新验证 CSV (仅 delayed_validation_enabled_ 时写入)
	        if (delayed_validation_enabled_ && csv_.is_open())
	        {
	            validation_csv_.open(
	                (csv_dir_ + "/" + self_name + "_delayed_update_validation.csv").c_str(),
	                std::ios::out | std::ios::trunc);
	            if (validation_csv_.is_open())
	            {
	                validation_csv_ << std::fixed << std::setprecision(9);
	                validation_csv_ << "time,self_id,target_id,obs_type,"
	                                << "accepted,replay_ok,found_target_update,reason,"
	                                << "obs_stamp,delayed_update_stamp,current_replay_stamp,"
	                                << "best_idx,cache_size,"
	                                << "delayed_err_before,delayed_err_after,delayed_err_delta,delayed_improved,"
	                                << "current_err_before,current_err_after,current_err_delta,current_improved,"
	                                << "delayed_before_x,delayed_before_y,delayed_before_z,"
	                                << "delayed_after_x,delayed_after_y,delayed_after_z,"
	                                << "delayed_gt_x,delayed_gt_y,delayed_gt_z,"
	                                << "current_before_x,current_before_y,current_before_z,"
	                                << "current_after_x,current_after_y,current_after_z,"
	                                << "current_gt_x,current_gt_y,current_gt_z,"
	                                << "residual_norm,nis,gain_weight,fusion_mode\n";
	                ROS_INFO("[DEKF] delayed update validation CSV: %s",
	                         (csv_dir_ + "/" + self_name + "_delayed_update_validation.csv").c_str());
	            }
	        }

	        // Trace CSV (可选)
	        openTraceCsv();
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
                   double nis,
                   double gain_consistency_error = std::numeric_limits<double>::quiet_NaN(),
                   const AdaptiveUwbInfo &adaptive_uwb = AdaptiveUwbInfo())
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
        const double recovery_reset_age =
            f.last_recovery_reset_stamp.isZero() || f.stamp.isZero()
                ? nan
                : (f.stamp - f.last_recovery_reset_stamp).toSec();

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
             << gain_consistency_error << ","
             << adaptive_uwb.noise << ","
             << adaptive_uwb.scale << ","
             << adaptive_uwb.anchor_age << ","
             << adaptive_uwb.reason << ","
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
             << anchor_age << ","
             << recovery_reset_age << ","
             << fusion_mode_ << ","
             << (enable_dgo_update_ ? 1 : 0) << ","
             << (enable_uwb_update_ ? 1 : 0) << ","
             << (enable_camera_update_ ? 1 : 0) << "\n";
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
                           double nis,
                           double gain_consistency_error = std::numeric_limits<double>::quiet_NaN(),
                           const AdaptiveUwbInfo &adaptive_uwb = AdaptiveUwbInfo())
    {
        const double obs_stamp =
            obs.stamp.isZero() ? std::numeric_limits<double>::quiet_NaN() : obs.stamp.toSec();
        logCsvRow(event, f, obsTypeName(obs.type), accepted, delayed, p_updated,
                  reason, obs_stamp, age, history_match_dt, residual_norm, nis,
                  gain_consistency_error, adaptive_uwb);
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

    bool hasAnyDirectionAnchor(const Filter &f) const
    {
        if (!require_direction_anchor_for_uwb_)
            return true;
        return !f.last_direction_anchor_stamp.isZero();
    }

    ros::Time latestReplayDirectionAnchorBefore(
        const std::deque<History_Record> &cache,
        int start_idx) const
    {
        ros::Time anchor_stamp;
        const int end = std::min(start_idx, static_cast<int>(cache.size()));
        for (int i = 0; i < end; ++i)
        {
            const History_Record &rec = cache[static_cast<size_t>(i)];
            for (const AppliedObservation &u : rec.updates)
            {
                if (u.obs.type == ObsType::DGO ||
                    u.obs.type == ObsType::CAMERA_BEARING)
                {
                    anchor_stamp = rec.stamp;
                }
            }
        }
        return anchor_stamp;
    }

    AdaptiveUwbInfo applyAdaptiveUwbNoise(const ros::Time &update_stamp,
                                          const ros::Time &direction_anchor_stamp,
                                          const Observation &obs,
                                          const Eigen::VectorXd &residual,
                                          Eigen::MatrixXd &R_meas) const
    {
        AdaptiveUwbInfo info;
        if (obs.type != ObsType::UWB_RANGE || R_meas.rows() < 1 || R_meas.cols() < 1)
            return info;

        double scale = 1.0;
        std::string reason = "base";
        const double nan = std::numeric_limits<double>::quiet_NaN();
        double anchor_age = nan;

        if (require_direction_anchor_for_uwb_)
        {
            if (direction_anchor_stamp.isZero() || update_stamp.isZero())
            {
                // no-anchor UWB should be rejected before reaching this function.
                // Keep this conservative fallback for defensive robustness.
                scale = std::max(scale, 100.0);
                reason += "+no_anchor_unexpected";
            }
            else
            {
                anchor_age = (update_stamp - direction_anchor_stamp).toSec();
                if (!std::isfinite(anchor_age) || anchor_age < 0.0)
                {
                    scale = std::max(scale, 25.0);
                    reason += "+invalid_anchor_age";
                }
                else if (anchor_age > uwb_anchor_max_age_)
                {
                    scale = std::max(scale, 10.0);
                    reason += "+stale_anchor";
                }
                else if (anchor_age > uwb_anchor_max_age_ * 0.5)
                {
                    scale = std::max(scale, 3.0);
                    reason += "+aging_anchor";
                }
            }
        }

        if (residual.size() >= 1 && std::isfinite(residual(0)))
        {
            const double abs_res = std::abs(residual(0));
            if (abs_res > uwb_residual_gate_ * 0.5 && abs_res <= uwb_residual_gate_)
            {
                const double ratio = abs_res / uwb_residual_gate_;
                const double residual_scale = 1.0 + (ratio - 0.5) * 4.0;
                scale *= residual_scale;
                reason += "+large_residual";
            }
        }

        info.scale = scale;
        info.noise = sigma_uwb_ * scale;
        info.anchor_age = anchor_age;
        info.reason = reason;
        R_meas(0, 0) = info.noise * info.noise;
        return info;
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
        rec.X_prior = f.X;
        rec.P_prior = f.P;
        rec.X_post = f.X;
        rec.P_post = f.P;
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
        f.last_recovery_reset_stamp = f.stamp;
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

    RecoveryResult tryDgoRecovery(Filter &f,
                                  const Observation &obs,
                                  bool delayed,
                                  double age,
                                  double history_match_dt,
                                  double residual_norm,
                                  double nis,
                                  const std::string &source_reason)
    {
        if (!enable_dgo_recovery_ || obs.type != ObsType::DGO ||
            !obs.position.allFinite() ||
            !std::isfinite(residual_norm) ||
            residual_norm < dgo_recovery_gate_)
        {
            return RecoveryResult::NONE;
        }

        if (!f.last_recovery_reset_stamp.isZero() && !f.stamp.isZero())
        {
            const double since_reset = (f.stamp - f.last_recovery_reset_stamp).toSec();
            if (std::isfinite(since_reset) &&
                since_reset >= 0.0 &&
                since_reset < dgo_recovery_cooldown_)
            {
                ++f.rejects;
                logObservationRow(delayed ? "delayed_update" : "current_update",
                                  f, obs, 0, delayed ? 1 : 0, 0,
                                  "dgo_recovery_cooldown_after_" + source_reason,
                                  age, history_match_dt, residual_norm, nis);
                return RecoveryResult::HANDLED_REJECTED;
            }
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
            return RecoveryResult::RESET;
        }
        else
        {
            ++f.rejects;
            logObservationRow(delayed ? "delayed_update" : "current_update",
                              f, obs, 0, delayed ? 1 : 0, 0,
                              "dgo_recovery_pending_after_" + source_reason,
                              age, history_match_dt, residual_norm, nis);
        }

        return RecoveryResult::HANDLED_REJECTED;
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

    // ── GT 插值函数 ──────────────────────────────────────
	    bool interpolateGt(const std::deque<GtWorldSample> &buf,
	                       int uav_id,
	                       const ros::Time &stamp,
	                       Eigen::Vector3d &p) const
	    {
	        if (buf.empty()) return false;

	        // 边界外: stamp 稍早于最早 → 用最近邻
	        if (stamp < buf.front().stamp)
	        {
	            double dt = (buf.front().stamp - stamp).toSec();
	            if (dt <= delayed_validation_gt_max_dt_)
	            {
	                p = buf.front().p[uav_id];
	                return true;
	            }
	            return false;
	        }

	        // 边界外: stamp 稍晚于最晚 → 用最近邻
	        if (stamp > buf.back().stamp)
	        {
	            double dt = (stamp - buf.back().stamp).toSec();
	            if (dt <= delayed_validation_gt_max_dt_)
	            {
	                p = buf.back().p[uav_id];
	                return true;
	            }
	            return false;
	        }

	        // 正常区间插值
	        for (size_t i = 1; i < buf.size(); ++i)
	        {
	            const auto &a = buf[i - 1];
	            const auto &b = buf[i];
	            if (a.stamp <= stamp && stamp <= b.stamp)
	            {
	                double dt = (b.stamp - a.stamp).toSec();
	                if (dt < 1e-9) { p = a.p[uav_id]; return true; }
	                double da = std::abs((stamp - a.stamp).toSec());
	                double db = std::abs((b.stamp - stamp).toSec());
	                if (std::min(da, db) > delayed_validation_gt_max_dt_) return false;
	                double alpha = (stamp - a.stamp).toSec() / dt;
	                p = (1.0 - alpha) * a.p[uav_id] + alpha * b.p[uav_id];
	                return true;
	            }
	        }
	        return false;
	    }

		    bool getRelativeGroundTruth(int self_id, int target_id,
		                                const ros::Time &stamp,
		                                Eigen::Vector3d &rel_gt) const
		    {
		        if (self_id < 0 || self_id >= 4 || target_id < 0 || target_id >= 4)
		            return false;
		        Eigen::Vector3d p_self, p_target;
		        if (!interpolateGt(model_gt_buffer_, self_id, stamp, p_self)) return false;
		        if (!interpolateGt(model_gt_buffer_, target_id, stamp, p_target)) return false;
		        rel_gt = p_target - p_self;
		        return true;
		    }

		    bool interpolateGtVelocity(const std::deque<GtWorldSample> &buf,
		                               int uav_id,
		                               const ros::Time &stamp,
		                               Eigen::Vector3d &v) const
		    {
		        if (buf.empty()) return false;
		        if (stamp < buf.front().stamp)
		        {
		            double dt = (buf.front().stamp - stamp).toSec();
		            if (dt <= delayed_validation_gt_max_dt_)
		            { v = buf.front().v[uav_id]; return true; }
		            return false;
		        }
		        if (stamp > buf.back().stamp)
		        {
		            double dt = (stamp - buf.back().stamp).toSec();
		            if (dt <= delayed_validation_gt_max_dt_)
		            { v = buf.back().v[uav_id]; return true; }
		            return false;
		        }
		        for (size_t i = 1; i < buf.size(); ++i)
		        {
		            const auto &a = buf[i - 1];
		            const auto &b = buf[i];
		            if (a.stamp <= stamp && stamp <= b.stamp)
		            {
		                double dt = (b.stamp - a.stamp).toSec();
		                if (dt < 1e-9) { v = a.v[uav_id]; return true; }
		                double da = std::abs((stamp - a.stamp).toSec());
		                double db = std::abs((b.stamp - stamp).toSec());
		                if (std::min(da, db) > delayed_validation_gt_max_dt_) return false;
		                double alpha = (stamp - a.stamp).toSec() / dt;
		                v = (1.0 - alpha) * a.v[uav_id] + alpha * b.v[uav_id];
		                return true;
		            }
		        }
		        return false;
		    }

		    bool getRelativeGroundTruthVelocity(int self_id, int target_id,
		                                         const ros::Time &stamp,
		                                         Eigen::Vector3d &rel_v) const
		    {
		        if (self_id < 0 || self_id >= 4 || target_id < 0 || target_id >= 4)
		            return false;
		        Eigen::Vector3d v_self, v_target;
		        if (!interpolateGtVelocity(model_gt_buffer_, self_id, stamp, v_self)) return false;
		        if (!interpolateGtVelocity(model_gt_buffer_, target_id, stamp, v_target)) return false;
		        rel_v = v_target - v_self;
		        return true;
		    }

	    uint64_t recordAcceptedObservation(History_Record &rec,
	                                       const Observation &obs,
	                                       bool delayed,
	                                       bool validation_target = false)
	    {
	        AppliedObservation applied;
	        applied.obs = obs;
	        applied.seq = obs_seq_++;
	        applied.delayed = delayed;
	        applied.validation_target = validation_target;
	        const uint64_t seq = applied.seq;
	        rec.updates.push_back(applied);

        std::sort(rec.updates.begin(), rec.updates.end(),
                  [](const AppliedObservation &a, const AppliedObservation &b) {
                      if (a.obs.stamp != b.obs.stamp)
                          return a.obs.stamp < b.obs.stamp;
                      return a.seq < b.seq;
                  });
        return seq;
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
            {
                const double pos_res = residual.head(3).norm();
                if (pos_res > dgo_residual_gate_)
                {
                    reason = "dgo_residual_gate_reject";
                    return false;
                }
                if (residual.size() >= 6)
                {
                    const double vel_res = residual.segment(3, 3).norm();
                    if (vel_res > dgo_velocity_residual_gate_)
                    {
                        reason = "dgo_velocity_residual_gate_reject";
                        return false;
                    }
                }
                return true;
            }

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
        if (!enable_dgo_update_)
            return;

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
        if (!enable_uwb_update_)
            return;
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
        if (!enable_camera_update_)
            return;
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

	    // ── 任务阶段回调 ────────────────────────────────────────
	    void stageCallback(const std_msgs::UInt8::ConstPtr &msg)
	    {
	        mission_stage_ = static_cast<int>(msg->data);
	    }

	    // ── GT 真值回调 ──────────────────────────────────────
	    int findModelIndex(const gazebo_msgs::ModelStates::ConstPtr &msg,
	                       const std::string &name) const
	    {
	        for (size_t i = 0; i < msg->name.size(); ++i)
	            if (msg->name[i] == name) return static_cast<int>(i);
	        return -1;
	    }

	    void gtModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr &msg)
		    {
		        if (!delayed_validation_enabled_ && !traceActive()) return;

		        GtWorldSample s;
		        s.stamp = ros::Time::now();

		        for (int id = 0; id < 4; ++id)
		        {
		            const std::string name = "iris_" + std::to_string(id);
		            const int idx = findModelIndex(msg, name);
		            if (idx < 0 ||
		                idx >= static_cast<int>(msg->pose.size()) ||
		                idx >= static_cast<int>(msg->twist.size()))
		                return;  // 本帧不完整, 丢弃
		            s.p[id] << msg->pose[idx].position.x,
		                      msg->pose[idx].position.y,
		                      msg->pose[idx].position.z;
		            s.v[id] << msg->twist[idx].linear.x,
		                      msg->twist[idx].linear.y,
		                      msg->twist[idx].linear.z;
		        }

		        model_gt_buffer_.push_back(s);
		        constexpr double kGtKeepSec = 10.0;
		        while (model_gt_buffer_.size() > 2 &&
		               (s.stamp - model_gt_buffer_.front().stamp).toSec() > kGtKeepSec)
		            model_gt_buffer_.pop_front();
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
                    const UpdateStatus status = applyCurrentUpdate(f, obs);
                    if (status == UpdateStatus::RECOVERY_RESET)
                    {
                        f.pending_obs.clear();
                        break;
                    }
                }
                else if (age < max_observation_delay_)
                {
                    const UpdateStatus status = applyDelayedUpdate(f, obs);
                    if (status == UpdateStatus::RECOVERY_RESET)
                    {
                        f.pending_obs.clear();
                        break;
                    }
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
        rec.X_prior = f.X;
        rec.P_prior = f.P;
        rec.X_post = f.X;
        rec.P_post = f.P;
        rec.A.setIdentity();
        rec.I_KH.setIdentity();
        rec.H.resize(0, 0);
        rec.K.resize(0, 0);
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
        rec.X_prior = rec.A * f.X;
        // P_{k+1|k} = A_k * P_k * A_k^T + G_k * Q * G_k^T
        rec.P_prior = rec.A * f.P * rec.A.transpose() + G * Q_ * G.transpose();
        symmetrizeCov(rec.P_prior);
        rec.X_post = rec.X_prior;
        rec.P_post = rec.P_prior;

        // 初始化延迟补偿缓存 (无更新时 I_KH=Identity, H/K 为空)
        rec.I_KH.setIdentity();
        rec.H.resize(0, 0);
        rec.K.resize(0, 0);

        f.X = rec.X_post;
        f.P = rec.P_post;
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
	    //  Trace 模块: 详细日志 (默认关闭)
	    // ═══════════════════════════════════════════════════════════

	    bool traceActive() const
	    {
	        if (!trace_enabled_) return false;
	        if (trace_dgo_only_ && fusion_mode_ != 1) return false;
	        if (csv_dir_.empty()) return false;
	        return true;
	    }

	    double safeRatio(double num, double den) const
	    {
	        if (!std::isfinite(num) || !std::isfinite(den) || std::abs(den) < 1e-12)
	            return std::numeric_limits<double>::quiet_NaN();
	        return num / den;
	    }

	    double safeCosine(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
	    {
	        const double na = a.norm();
	        const double nb = b.norm();
	        if (na < 1e-12 || nb < 1e-12)
	            return std::numeric_limits<double>::quiet_NaN();
	        return a.dot(b) / (na * nb);
	    }

	    double getK(const Eigen::MatrixXd &K, int r, int c) const
	    {
	        if (r < K.rows() && c < K.cols()) return K(r, c);
	        return std::numeric_limits<double>::quiet_NaN();
	    }

	    bool getLatestDgoForTarget(int target_id,
	                               const ros::Time &now,
	                               Eigen::Vector3d &dgo_rel,
	                               double &age) const
	    {
	        if (target_id < 0 || target_id >= uav_num_ ||
	            uav_id_ < 0 || uav_id_ >= uav_num_)
	            return false;
	        const auto &target_cache = dgo_cache_[target_id];
	        const auto &self_cache   = dgo_cache_[uav_id_];
	        if (target_cache.empty() || self_cache.empty()) return false;

	        const DGOSample &t = target_cache.back();
	        const DGOSample &s = self_cache.back();

	        // DGO world-frame position → self-to-target relative
	        // rel = (init_off_target + dgo_target) - (init_off_self + dgo_self)
	        const Eigen::Vector3d t_world(t.msg.pose.pose.position.x,
	                                      t.msg.pose.pose.position.y,
	                                      t.msg.pose.pose.position.z);
	        const Eigen::Vector3d s_world(s.msg.pose.pose.position.x,
	                                      s.msg.pose.pose.position.y,
	                                      s.msg.pose.pose.position.z);

	        dgo_rel = (initial_offsets_[target_id] + t_world)
	                - (initial_offsets_[uav_id_] + s_world);

	        // Conservative age: max of both estimates' age
	        const double age_self   = (now - s.stamp).toSec();
	        const double age_target = (now - t.stamp).toSec();
	        age = std::isfinite(age_self) && std::isfinite(age_target)
	              ? std::max(age_self, age_target) : std::numeric_limits<double>::quiet_NaN();
	        return std::isfinite(age);
	    }

	    void openTraceCsv()
	    {
	        if (!traceActive()) return;
	        if (!ensureDirectory(csv_dir_))
	        {
	            ROS_WARN("[DEKF] trace: cannot create csv_dir=%s", csv_dir_.c_str());
	            return;
	        }

	        const std::string self_name = "iris_" + std::to_string(uav_id_);

	        const std::string delayed_path = csv_dir_ + "/" + self_name + "_delayed_update_trace.csv";
	        delayed_trace_csv_.open(delayed_path.c_str(), std::ios::out | std::ios::trunc);
	        if (!delayed_trace_csv_.is_open())
	        {
	            ROS_WARN("[DEKF] trace: cannot open %s", delayed_path.c_str());
	        }
	        else
	        {
	            delayed_trace_csv_ << std::fixed << std::setprecision(9);
		            delayed_trace_csv_
		                << "time,self_id,target_id,stage,"
		                << "obs_stamp,filter_stamp,age,hist_stamp,history_match_dt,best_idx,cache_size,"
		                << "z_x,z_y,z_z,"
		                << "update_before_x,update_before_y,update_before_z,"
		                << "pred_x,pred_y,pred_z,"
		                << "res_x,res_y,res_z,res_norm,pos_res_norm,vel_res_norm,"
	                << "Ppx,Ppy,Ppz,Pvx,Pvy,Pvz,"
	                << "Sx,Sy,Sz,"
		                << "Kpx,Kpy,Kpz,Kvx,Kvy,Kvz,Kvv_x,Kvv_y,Kvv_z,"
	                << "x_before,y_before,z_before,vx_before,vy_before,vz_before,"
	                << "x_after,y_after,z_after,vx_after,vy_after,vz_after,"
	                << "delta_x,delta_y,delta_z,delta_vx,delta_vy,delta_vz,"
	                << "delta_pos_norm,delta_vel_norm,"
	                << "gt_x,gt_y,gt_z,"
	                << "err_before_x,err_before_y,err_before_z,"
	                << "err_after_x,err_after_y,err_after_z,"
	                << "err_before_norm,err_after_norm,err_delta,"
	                << "correction_dot_error,correction_cos_error,"
	                << "correction_norm_over_residual,"
	                << "gain_weight\n";
	            ROS_INFO("[DEKF] trace delayed_update CSV: %s", delayed_path.c_str());
	        }

	        const std::string replay_path = csv_dir_ + "/" + self_name + "_replay_trace.csv";
	        replay_trace_csv_.open(replay_path.c_str(), std::ios::out | std::ios::trunc);
	        if (!replay_trace_csv_.is_open())
	        {
	            ROS_WARN("[DEKF] trace: cannot open %s", replay_path.c_str());
	        }
	        else
	        {
	            replay_trace_csv_ << std::fixed << std::setprecision(9);
	            replay_trace_csv_
	                << "time,self_id,target_id,stage,"
	                << "obs_stamp,hist_stamp,current_stamp,"
	                << "best_idx,cache_size,replay_nodes,replay_duration,"
	                << "hist_delta_pos_norm,hist_delta_vel_norm,"
	                << "current_delta_x,current_delta_y,current_delta_z,"
	                << "current_delta_vx,current_delta_vy,current_delta_vz,"
	                << "current_delta_pos_norm,current_delta_vel_norm,"
	                << "propagation_gain_pos,propagation_gain_vel,"
	                << "current_before_x,current_before_y,current_before_z,"
	                << "current_before_vx,current_before_vy,current_before_vz,"
	                << "current_after_x,current_after_y,current_after_z,"
	                << "current_after_vx,current_after_vy,current_after_vz,"
	                << "current_gt_x,current_gt_y,current_gt_z,"
	                << "current_err_before_x,current_err_before_y,current_err_before_z,"
	                << "current_err_after_x,current_err_after_y,current_err_after_z,"
	                << "current_err_before_norm,current_err_after_norm,current_err_delta\n";
	            ROS_INFO("[DEKF] trace replay CSV: %s", replay_path.c_str());
	        }

	        const std::string state_path = csv_dir_ + "/" + self_name + "_state_trace.csv";
	        state_trace_csv_.open(state_path.c_str(), std::ios::out | std::ios::trunc);
	        if (!state_trace_csv_.is_open())
	        {
	            ROS_WARN("[DEKF] trace: cannot open %s", state_path.c_str());
	        }
	        else
	        {
	            state_trace_csv_ << std::fixed << std::setprecision(9);
	            state_trace_csv_
	                << "time,self_id,target_id,stage,"
	                << "x,y,z,vx,vy,vz,"
	                << "Pxx,Pyy,Pzz,Pvxvx,Pvyvy,Pvzvz,"
	                << "gt_x,gt_y,gt_z,"
	                << "gt_vx,gt_vy,gt_vz,"
	                << "latest_dgo_x,latest_dgo_y,latest_dgo_z,latest_dgo_age,"
	                << "err_gt_x,err_gt_y,err_gt_z,err_gt_norm,"
	                << "err_dgo_x,err_dgo_y,err_dgo_z,err_dgo_norm,"
	                << "est_rel_speed_norm,gt_rel_speed_norm,"
	                << "vel_err_x,vel_err_y,vel_err_z,vel_err_norm\n";
	            ROS_INFO("[DEKF] trace state CSV: %s", state_path.c_str());
	        }
	    }

	    void writeStateTrace(const Filter &f)
	    {
	        if (!state_trace_csv_.is_open()) return;
	        if (!traceActive()) return;

	        const double t = f.stamp.isZero() ? ros::Time::now().toSec() : f.stamp.toSec();
	        const int target = f.target_id;
	        const double nan = std::numeric_limits<double>::quiet_NaN();

	        // GT
	        Eigen::Vector3d gt_rel = Eigen::Vector3d::Constant(nan);
	        (void)getRelativeGroundTruth(uav_id_, target, f.stamp, gt_rel);
	        Eigen::Vector3d gt_rel_v = Eigen::Vector3d::Constant(nan);
	        (void)getRelativeGroundTruthVelocity(uav_id_, target, f.stamp, gt_rel_v);

	        // Latest DGO
	        Eigen::Vector3d dgo_rel = Eigen::Vector3d::Constant(nan);
	        double dgo_age = nan;
	        (void)getLatestDgoForTarget(target, f.stamp, dgo_rel, dgo_age);

	        // Errors
	        const Eigen::Vector3d pos = f.X.segment<3>(0);
	        const Eigen::Vector3d vel = f.X.segment<3>(3);
	        const Eigen::Vector3d err_gt = pos - gt_rel;
	        const Eigen::Vector3d err_dgo = pos - dgo_rel;
	        const double err_gt_norm = err_gt.allFinite() ? err_gt.norm() : nan;
	        const double err_dgo_norm = err_dgo.allFinite() ? err_dgo.norm() : nan;
	        const Eigen::Vector3d vel_err = vel - gt_rel_v;
	        const double vel_err_norm = vel_err.allFinite() ? vel_err.norm() : nan;
	        const double est_speed = vel.allFinite() ? vel.norm() : nan;
	        const double gt_speed = gt_rel_v.allFinite() ? gt_rel_v.norm() : nan;

	        state_trace_csv_
	            << t << ","
	            << uav_id_ << "," << target << "," << mission_stage_ << ","
	            << pos(0) << "," << pos(1) << "," << pos(2) << ","
	            << vel(0) << "," << vel(1) << "," << vel(2) << ","
	            << f.P(0,0) << "," << f.P(1,1) << "," << f.P(2,2) << ","
	            << f.P(3,3) << "," << f.P(4,4) << "," << f.P(5,5) << ","
	            << gt_rel(0) << "," << gt_rel(1) << "," << gt_rel(2) << ","
	            << gt_rel_v(0) << "," << gt_rel_v(1) << "," << gt_rel_v(2) << ","
	            << dgo_rel(0) << "," << dgo_rel(1) << "," << dgo_rel(2) << "," << dgo_age << ","
	            << err_gt(0) << "," << err_gt(1) << "," << err_gt(2) << "," << err_gt_norm << ","
	            << err_dgo(0) << "," << err_dgo(1) << "," << err_dgo(2) << "," << err_dgo_norm << ","
	            << est_speed << "," << gt_speed << ","
	            << vel_err(0) << "," << vel_err(1) << "," << vel_err(2) << "," << vel_err_norm << "\n";
	    }

	    void writeDelayedUpdateTrace(
	        double t,
	        const ros::Time &obs_stamp,
	        const ros::Time &filter_stamp,
	        const ros::Time &hist_stamp,
	        int target_id,
	        double age,
	        double history_match_dt,
	        int best_idx,
	        int cache_size,
	        const Vector6d &x_before,
	        const Vector6d &x_after,
	        const Eigen::VectorXd &Zk,
	        const Eigen::VectorXd &residual,
	        const Eigen::MatrixXd &K_eff,
	        const Eigen::MatrixXd &S,
	        const Vector6d &P_diag,
	        double gain_weight_val)
	    {
	        if (!delayed_trace_csv_.is_open()) return;
	        if (!traceActive()) return;

	        const double nan = std::numeric_limits<double>::quiet_NaN();

	        // GT
	        Eigen::Vector3d gt_rel = Eigen::Vector3d::Constant(nan);
	        (void)getRelativeGroundTruth(uav_id_, target_id, hist_stamp, gt_rel);

	        const Eigen::Vector3d delta_pos = x_after.head<3>() - x_before.head<3>();
	        const Eigen::Vector3d delta_vel = x_after.tail<3>() - x_before.tail<3>();
	        const double delta_pos_norm = delta_pos.norm();
	        const double delta_vel_norm = delta_vel.norm();

	        Eigen::Vector3d z3 = Eigen::Vector3d::Constant(nan);
	        Eigen::Vector3d res3 = Eigen::Vector3d::Constant(nan);
	        if (Zk.size() >= 3) z3 = Zk.head<3>();
	        if (residual.size() >= 3) res3 = residual.head<3>();

	        const Eigen::Vector3d pred3 = z3 - res3;

	        // P diag
	        const double Ppx = P_diag.size() > 0 ? P_diag(0) : nan;
	        const double Ppy = P_diag.size() > 1 ? P_diag(1) : nan;
	        const double Ppz = P_diag.size() > 2 ? P_diag(2) : nan;

	        // S diag
	        const double Sx = S.rows() > 0 ? S(0,0) : nan;
	        const double Sy = S.rows() > 1 ? S(1,1) : nan;
	        const double Sz = S.rows() > 2 ? S(2,2) : nan;

	        // K diag
	        const double Kpx = getK(K_eff, 0, 0);
	        const double Kpy = getK(K_eff, 1, 1);
	        const double Kpz = getK(K_eff, 2, 2);
	        const double Kvx = getK(K_eff, 3, 0);
	        const double Kvy = getK(K_eff, 4, 1);
	        const double Kvz = getK(K_eff, 5, 2);

	        const Eigen::Vector3d err_before = x_before.head<3>() - gt_rel;
	        const Eigen::Vector3d err_after  = x_after.head<3>()  - gt_rel;
	        const double err_before_norm = err_before.allFinite() ? err_before.norm() : nan;
	        const double err_after_norm  = err_after.allFinite()  ? err_after.norm()  : nan;
	        const double err_delta = (err_before.allFinite() && err_after.allFinite())
	                                 ? (err_after_norm - err_before_norm) : nan;

	        const double correction_dot = delta_pos.allFinite() && err_before.allFinite()
	                                      ? delta_pos.dot(err_before) : nan;
	        const double correction_cos = safeCosine(delta_pos, err_before);

        const double residual_norm = residual.allFinite() ? residual.norm() : nan;
        const double pos_res_norm = residual.size() >= 3 && residual.head(3).allFinite()
                                    ? residual.head(3).norm() : nan;
        const double vel_res_norm = residual.size() >= 6 && residual.segment(3,3).allFinite()
                                    ? residual.segment(3,3).norm() : nan;

        // Kvv: velocity-residual → velocity-state gain (6D 模式下 velocity diagonal)
        const double Kvv_x = getK(K_eff, 3, 3);
        const double Kvv_y = getK(K_eff, 4, 4);
        const double Kvv_z = getK(K_eff, 5, 5);

        const double corr_over_res = safeRatio(delta_pos_norm, residual_norm);

	        delayed_trace_csv_
	            << t << ","
	            << uav_id_ << "," << target_id << "," << mission_stage_ << ","
	            << obs_stamp.toSec() << "," << filter_stamp.toSec() << "," << age << ","
	            << hist_stamp.toSec() << "," << history_match_dt << "," << best_idx << "," << cache_size << ","
	            << z3(0) << "," << z3(1) << "," << z3(2) << ","
	            << x_before(0) << "," << x_before(1) << "," << x_before(2) << ","
	            << pred3(0) << "," << pred3(1) << "," << pred3(2) << ","
            << res3(0) << "," << res3(1) << "," << res3(2) << "," << residual_norm << ","
            << pos_res_norm << "," << vel_res_norm << ","
            << Ppx << "," << Ppy << "," << Ppz << ","
            << P_diag(3) << "," << P_diag(4) << "," << P_diag(5) << ","
            << Sx << "," << Sy << "," << Sz << ","
            << Kpx << "," << Kpy << "," << Kpz << ","
            << Kvx << "," << Kvy << "," << Kvz << ","
            << Kvv_x << "," << Kvv_y << "," << Kvv_z << ","
	            << x_before(0) << "," << x_before(1) << "," << x_before(2) << ","
	            << x_before(3) << "," << x_before(4) << "," << x_before(5) << ","
	            << x_after(0) << "," << x_after(1) << "," << x_after(2) << ","
	            << x_after(3) << "," << x_after(4) << "," << x_after(5) << ","
	            << delta_pos(0) << "," << delta_pos(1) << "," << delta_pos(2) << ","
	            << delta_vel(0) << "," << delta_vel(1) << "," << delta_vel(2) << ","
	            << delta_pos_norm << "," << delta_vel_norm << ","
	            << gt_rel(0) << "," << gt_rel(1) << "," << gt_rel(2) << ","
	            << err_before(0) << "," << err_before(1) << "," << err_before(2) << ","
	            << err_after(0) << "," << err_after(1) << "," << err_after(2) << ","
	            << err_before_norm << "," << err_after_norm << "," << err_delta << ","
	            << correction_dot << "," << correction_cos << ","
	            << corr_over_res << ","
	            << gain_weight_val << "\n";
	    }

	    void writeReplayTrace(
	        const Filter &f,
	        const Observation &obs,
	        const ReplayTraceContext &ctx,
	        const Vector6d &current_before,
	        const Vector6d &current_after,
	        int replay_nodes,
	        double replay_duration)
	    {
	        if (!replay_trace_csv_.is_open()) return;
	        if (!traceActive()) return;

	        const double t = f.stamp.isZero() ? ros::Time::now().toSec() : f.stamp.toSec();
	        const double nan = std::numeric_limits<double>::quiet_NaN();

	        // GT for current state
	        Eigen::Vector3d current_gt = Eigen::Vector3d::Constant(nan);
	        (void)getRelativeGroundTruth(uav_id_, f.target_id, f.stamp, current_gt);

	        const Eigen::Vector3d hist_delta = ctx.hist_after.head<3>() - ctx.hist_before.head<3>();
	        const Eigen::Vector3d hist_delta_vel = ctx.hist_after.tail<3>() - ctx.hist_before.tail<3>();
	        const double hist_delta_pos_norm = hist_delta.norm();
	        const double hist_delta_vel_norm = hist_delta_vel.norm();

	        const Eigen::Vector3d current_delta = current_after.head<3>() - current_before.head<3>();
	        const Eigen::Vector3d current_delta_vel = current_after.tail<3>() - current_before.tail<3>();
	        const double current_delta_pos_norm = current_delta.norm();
	        const double current_delta_vel_norm = current_delta_vel.norm();

	        const double prop_gain_pos = safeRatio(current_delta_pos_norm, hist_delta_pos_norm);
	        const double prop_gain_vel = safeRatio(current_delta_vel_norm, hist_delta_vel_norm);

	        const Eigen::Vector3d err_before = current_before.head<3>() - current_gt;
	        const Eigen::Vector3d err_after  = current_after.head<3>()  - current_gt;
	        const double err_before_norm = err_before.allFinite() ? err_before.norm() : nan;
	        const double err_after_norm  = err_after.allFinite()  ? err_after.norm()  : nan;
	        const double err_delta_val = (std::isfinite(err_before_norm) && std::isfinite(err_after_norm))
	                                     ? (err_after_norm - err_before_norm) : nan;

	        replay_trace_csv_
	            << t << ","
	            << uav_id_ << "," << f.target_id << "," << mission_stage_ << ","
	            << ctx.obs_stamp.toSec() << "," << ctx.hist_stamp.toSec() << ","
	            << ctx.current_stamp.toSec() << ","
	            << ctx.best_idx << "," << ctx.cache_size << "," << replay_nodes << "," << replay_duration << ","
	            << hist_delta_pos_norm << "," << hist_delta_vel_norm << ","
	            << current_delta(0) << "," << current_delta(1) << "," << current_delta(2) << ","
	            << current_delta_vel(0) << "," << current_delta_vel(1) << "," << current_delta_vel(2) << ","
	            << current_delta_pos_norm << "," << current_delta_vel_norm << ","
	            << prop_gain_pos << "," << prop_gain_vel << ","
	            << current_before(0) << "," << current_before(1) << "," << current_before(2) << ","
	            << current_before(3) << "," << current_before(4) << "," << current_before(5) << ","
	            << current_after(0) << "," << current_after(1) << "," << current_after(2) << ","
	            << current_after(3) << "," << current_after(4) << "," << current_after(5) << ","
	            << current_gt(0) << "," << current_gt(1) << "," << current_gt(2) << ","
	            << err_before(0) << "," << err_before(1) << "," << err_before(2) << ","
	            << err_after(0) << "," << err_after(1) << "," << err_after(2) << ","
	            << err_before_norm << "," << err_after_norm << "," << err_delta_val << "\n";
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
                if (use_dgo_velocity_ && obs.velocity.allFinite())
                {
                    // position + velocity
                    Z.resize(6);
                    Z << obs.position(0), obs.position(1), obs.position(2),
                         obs.velocity(0), obs.velocity(1), obs.velocity(2);
                    residual = Z - x;
                    H.resize(6, 6);
                    H.setIdentity();
                    R_meas.resize(6, 6);
                    R_meas.setZero();
                    R_meas(0, 0) = sigma_px_ * sigma_px_;
                    R_meas(1, 1) = sigma_py_ * sigma_py_;
                    R_meas(2, 2) = sigma_pz_ * sigma_pz_;
                    const double sv2 = dgo_velocity_noise_std_ * dgo_velocity_noise_std_;
                    R_meas(3, 3) = sv2;
                    R_meas(4, 4) = sv2;
                    R_meas(5, 5) = sv2;
                }
                else
                {
                    Z.resize(3);
                    Z << obs.position(0), obs.position(1), obs.position(2);
                    residual = Z - x.segment(0, 3);
                    H.resize(3, 6);
                    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
                    H.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
                    R_meas.resize(3, 3);
                    R_meas = R_.block(0, 0, 3, 3);
                }
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
                // 基础 R_meas = sigma_uwb_^2, 自适应膨胀交由调用方处理
                R_meas.resize(1, 1);
                R_meas(0, 0) = sigma_uwb_ * sigma_uwb_;
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

    bool isBoundedCovariance(const Matrix6d &P) const
    {
        if (!isUsableCovariance(P))
            return false;

        Matrix6d P_sym = 0.5 * (P + P.transpose());
        for (int i = 0; i < 3; ++i)
        {
            if (P_sym(i, i) > max_position_cov_)
                return false;
        }
        for (int i = 3; i < 6; ++i)
        {
            if (P_sym(i, i) > max_velocity_cov_)
                return false;
        }
        return true;
    }

	    bool replayCacheFrom(std::deque<History_Record> &cache,
	                         int start_idx,
	                         bool gate_observation,
	                         uint64_t gated_seq,
	                         std::string &fail_reason,
	                         double &fail_residual_norm,
	                         double &fail_nis,
	                         bool &failed_on_gated_observation,
	                         AdaptiveUwbInfo &gated_adaptive_uwb,
	                         DelayedUpdateValidation *validation = nullptr,
	                         ReplayTraceContext *trace_ctx = nullptr)
    {
        fail_reason.clear();
        fail_residual_norm = std::numeric_limits<double>::quiet_NaN();
        fail_nis = std::numeric_limits<double>::quiet_NaN();
        failed_on_gated_observation = false;
        gated_adaptive_uwb = AdaptiveUwbInfo();
        bool saw_gated_observation = !gate_observation;

        if (start_idx < 0 || start_idx >= static_cast<int>(cache.size()))
        {
            fail_reason = "replay_invalid_start_idx";
            return false;
        }

        ros::Time replay_direction_anchor_stamp =
            latestReplayDirectionAnchorBefore(cache, start_idx);

        for (int i = start_idx; i < static_cast<int>(cache.size()); ++i)
        {
            History_Record &rec = cache[static_cast<size_t>(i)];

            if (i > start_idx)
            {
                const History_Record &prev = cache[static_cast<size_t>(i - 1)];
                const double dt = (rec.stamp - prev.stamp).toSec();
                if (!std::isfinite(dt) || dt < 1e-6)
                {
                    fail_reason = "replay_invalid_dt";
                    return false;
                }

                Eigen::Matrix<double,6,3> G;
                G.block<3,3>(0,0) = 0.5 * Eigen::Matrix3d::Identity() * dt * dt;
                G.block<3,3>(3,0) = Eigen::Matrix3d::Identity() * dt;

                rec.X_prior = rec.A * prev.X_post;
                rec.P_prior = rec.A * prev.P_post * rec.A.transpose() + G * Q_ * G.transpose();
                symmetrizeCov(rec.P_prior);
                if (!rec.X_prior.allFinite() || !isUsableCovariance(rec.P_prior))
                {
                    fail_reason = "replay_prior_invalid";
                    return false;
                }
            }

            rec.X_post = rec.X_prior;
            rec.P_post = rec.P_prior;
            rec.I_KH.setIdentity();
            rec.H.resize(0, 0);
            rec.K.resize(0, 0);

            for (const AppliedObservation &u : rec.updates)
            {
                const Observation &obs_i = u.obs;
                const bool is_gated_observation =
                    gate_observation && u.seq == gated_seq;
                if (is_gated_observation)
                    saw_gated_observation = true;

                Eigen::VectorXd Zk;
                Eigen::VectorXd residual;
                Eigen::MatrixXd Hk;
                Eigen::MatrixXd R_meas;

                if (!buildObsModel(rec.X_post, obs_i, Zk, residual, Hk, R_meas))
                {
                    failed_on_gated_observation = is_gated_observation;
                    fail_reason = "replay_build_obs_model_failed";
                    return false;
                }

                const double residual_norm = residual.norm();
                AdaptiveUwbInfo adaptive_uwb;
                if (obs_i.type == ObsType::UWB_RANGE)
                {
                    if (require_direction_anchor_for_uwb_ &&
                        replay_direction_anchor_stamp.isZero())
                    {
                        failed_on_gated_observation = is_gated_observation;
                        fail_residual_norm = residual_norm;
                        fail_reason = "uwb_no_direction_anchor";
                        if (is_gated_observation)
                        {
                            gated_adaptive_uwb.reason = "no_anchor_reject";
                            gated_adaptive_uwb.scale = std::numeric_limits<double>::infinity();
                            gated_adaptive_uwb.noise = std::numeric_limits<double>::infinity();
                        }
                        return false;
                    }

                    adaptive_uwb =
                        applyAdaptiveUwbNoise(rec.stamp,
                                              replay_direction_anchor_stamp,
                                              obs_i,
                                              residual,
                                              R_meas);
                    if (is_gated_observation)
                        gated_adaptive_uwb = adaptive_uwb;
                }

                if (is_gated_observation)
                {
                    fail_residual_norm = residual_norm;
                    std::string gate_reason;
                    if (!passHardResidualGate(obs_i, residual, gate_reason))
                    {
                        failed_on_gated_observation = true;
                        fail_reason = gate_reason;
                        return false;
                    }
                }

                Eigen::MatrixXd S = Hk * rec.P_post * Hk.transpose() + R_meas;
                Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
                if (!lu.isInvertible())
                {
                    failed_on_gated_observation = is_gated_observation;
                    fail_residual_norm = residual_norm;
                    fail_reason = "replay_innovation_cov_not_invertible";
                    return false;
                }

                const Eigen::MatrixXd S_inv = lu.inverse();
                const double nis = residual.transpose() * S_inv * residual;
                if (is_gated_observation)
                {
                    fail_residual_norm = residual_norm;
                    fail_nis = nis;
                    const double nis_gate = nisGateForObservation(obs_i, residual.size());
                    if (nis > nis_gate)
                    {
                        failed_on_gated_observation = true;
                        fail_reason = "nis_reject";
                        return false;
                    }
                }

                Eigen::MatrixXd K = rec.P_post * Hk.transpose() * S_inv;
	                const double gain_weight = u.delayed ? delayed_update_gain_weight_ : 1.0;
	                Eigen::MatrixXd K_eff = gain_weight * K;

		                // ── validation tracking ─────────────────────
		                if (validation && validation->enabled && u.seq == gated_seq)
		                {
		                    validation->delayed_update_before = rec.X_post;
		                    validation->delayed_update_stamp = rec.stamp;
		                    validation->residual_norm = residual.norm();
		                    validation->nis = nis;
		                    validation->gain_weight = gain_weight;
		                    validation->found_target_update = true;
		                }

		                // ── trace: delayed update ───────────────────
		                if (trace_ctx && trace_ctx->enabled && u.seq == gated_seq)
		                {
		                    trace_ctx->found_target_update = true;
		                    trace_ctx->hist_before = rec.X_post;
		                    trace_ctx->hist_stamp = rec.stamp;

		                    // 在 apply update 前记录 delayed_update_trace
		                    Vector6d x_before = rec.X_post;
		                    (void)x_before; // used after update
		                }

		                rec.X_post += K_eff * residual;

		                if (validation && validation->enabled && u.seq == gated_seq)
		                {
		                    validation->delayed_update_after = rec.X_post;
		                }

		                // ── trace: delayed update after ────────────
		                if (trace_ctx && trace_ctx->enabled && u.seq == gated_seq)
			                {
			                    trace_ctx->hist_after = rec.X_post;

			                    // 写 delayed update trace
			                    Vector6d x_before = trace_ctx->hist_before;
			                    Vector6d x_after = rec.X_post;

			                    // 构建 P_diag 向量 (协方差对角)
			                    Vector6d P_diag;
			                    P_diag << rec.P_post(0,0), rec.P_post(1,1), rec.P_post(2,2),
			                              rec.P_post(3,3), rec.P_post(4,4), rec.P_post(5,5);

			                    const double trace_t = rec.stamp.isZero()
			                        ? ros::Time::now().toSec() : rec.stamp.toSec();

			                    writeDelayedUpdateTrace(
			                        trace_t,
			                        obs_i.stamp,
			                        trace_ctx->current_stamp,  // filter_stamp: 当前滤波器时间
			                        rec.stamp,                 // hist_stamp: 缓存节点时间
			                        obs_i.target_id,
			                        (trace_ctx->current_stamp - obs_i.stamp).toSec(),  // age
			                        (rec.stamp - obs_i.stamp).toSec(),  // history_match_dt
			                        start_idx,       // best_idx
			                        static_cast<int>(cache.size()),  // cache_size
			                        x_before,
			                        x_after,
			                        Zk,
			                        residual,
			                        K_eff,
			                        S,
			                        P_diag,
			                        gain_weight);
		                }
                Matrix6d I_KH = Matrix6d::Identity() - K_eff * Hk;
                rec.P_post =
                    I_KH * rec.P_post * I_KH.transpose() +
                    K_eff * R_meas * K_eff.transpose();
                symmetrizeCov(rec.P_post);

                if (!rec.X_post.allFinite() || !isUsableCovariance(rec.P_post))
                {
                    failed_on_gated_observation = is_gated_observation;
                    fail_reason = "replay_post_invalid";
                    return false;
                }

                rec.I_KH = I_KH * rec.I_KH;
                rec.H = Hk;
                rec.K = K_eff;

                if (obs_i.type == ObsType::DGO ||
                    obs_i.type == ObsType::CAMERA_BEARING)
                {
                    replay_direction_anchor_stamp = rec.stamp;
                }
            }
        }

        if (!saw_gated_observation)
        {
            fail_reason = "replay_gated_observation_not_found";
            return false;
        }

        return true;
    }

    // ═══════════════════════════════════════════════════════════
    //  观测更新函数
    // ═══════════════════════════════════════════════════════════

    // 当前时刻 EKF 更新: 观测时间 ≈ 当前预测时间, 直接执行标准 EKF 更新
    UpdateStatus applyCurrentUpdate(Filter &f, const Observation &obs)
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
            return UpdateStatus::REJECTED;
        }

        const double residual_norm = residual.norm();
        AdaptiveUwbInfo adaptive_uwb;
        if (obs.type == ObsType::UWB_RANGE)
        {
            if (!hasAnyDirectionAnchor(f))
            {
                adaptive_uwb.reason = "no_anchor_reject";
                adaptive_uwb.scale = std::numeric_limits<double>::infinity();
                adaptive_uwb.noise = std::numeric_limits<double>::infinity();

                ++f.rejects;
                logObservationRow("current_update", f, obs, 0, 0, 0,
                                  "uwb_no_direction_anchor", age,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  residual_norm,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  adaptive_uwb);
                return UpdateStatus::REJECTED;
            }

            adaptive_uwb = applyAdaptiveUwbNoise(f.stamp,
                                                 f.last_direction_anchor_stamp,
                                                 obs,
                                                 residual,
                                                 R_meas);
        }

        std::string gate_reason;
        if (!passHardResidualGate(obs, residual, gate_reason))
        {
            const RecoveryResult recovery =
                tryDgoRecovery(f, obs, false, age,
                               std::numeric_limits<double>::quiet_NaN(),
                               residual_norm,
                               std::numeric_limits<double>::quiet_NaN(),
                               gate_reason);
            if (recovery == RecoveryResult::RESET)
            {
                return UpdateStatus::RECOVERY_RESET;
            }
            if (recovery == RecoveryResult::HANDLED_REJECTED)
            {
                return UpdateStatus::REJECTED;
            }

            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              gate_reason, age,
                              std::numeric_limits<double>::quiet_NaN(),
                              residual_norm,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              adaptive_uwb);
            return UpdateStatus::REJECTED;
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
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              adaptive_uwb);
            return UpdateStatus::REJECTED;
        }
        const Eigen::MatrixXd S_inv = lu.inverse();
        const double nis = residual.transpose() * S_inv * residual;
        const double nis_gate = nisGateForObservation(obs, residual.size());
        if (nis > nis_gate)
        {
            const RecoveryResult recovery =
                tryDgoRecovery(f, obs, false, age,
                               std::numeric_limits<double>::quiet_NaN(),
                               residual_norm, nis, "nis_reject");
            if (recovery == RecoveryResult::RESET)
            {
                return UpdateStatus::RECOVERY_RESET;
            }
            if (recovery == RecoveryResult::HANDLED_REJECTED)
            {
                return UpdateStatus::REJECTED;
            }

            ++f.rejects;
            logObservationRow("current_update", f, obs, 0, 0, 0,
                              "nis_reject", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              residual_norm,
                              nis,
                              std::numeric_limits<double>::quiet_NaN(),
                              adaptive_uwb);
            return UpdateStatus::REJECTED;
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
            recordAcceptedObservation(f.cache.back(), obs, false);
            f.cache.back().X_post = f.X;
            f.cache.back().P_post = f.P;
        }

        logObservationRow("current_update", f, obs, 1, 0, 1,
                          "accepted", age,
                          std::numeric_limits<double>::quiet_NaN(),
                          residual_norm,
                          nis,
                          std::numeric_limits<double>::quiet_NaN(),
                          adaptive_uwb);
        return UpdateStatus::ACCEPTED;
	    }

	    // ── delayed update validation ─────────────────────────
	    void writeDelayedValidationCsv(const DelayedUpdateValidation &val)
	    {
	        if (!val.enabled) return;
	        if (!validation_csv_.is_open()) return;

	        // GT 误差
	        auto computeErr = [](const Vector6d &x, const Eigen::Vector3d &gt, bool has_gt) -> double {
	            if (!has_gt) return std::numeric_limits<double>::quiet_NaN();
	            return (x.head<3>() - gt).norm();
	        };

	        double delayed_err_before = computeErr(val.delayed_update_before, val.delayed_update_gt, val.has_delayed_update_gt && val.found_target_update);
	        double delayed_err_after = computeErr(val.delayed_update_after, val.delayed_update_gt, val.has_delayed_update_gt && val.found_target_update);
	        double delayed_err_delta = (std::isfinite(delayed_err_before) && std::isfinite(delayed_err_after))
	                                        ? delayed_err_after - delayed_err_before
	                                        : std::numeric_limits<double>::quiet_NaN();
	        int delayed_improved = std::isfinite(delayed_err_delta) ? (delayed_err_delta < 0.0 ? 1 : 0) : -1;

	        double current_err_before = computeErr(val.current_replay_before, val.current_replay_gt, val.has_current_replay_gt);
	        double current_err_after = computeErr(val.current_replay_after, val.current_replay_gt, val.has_current_replay_gt);
	        double current_err_delta = (std::isfinite(current_err_before) && std::isfinite(current_err_after))
	                                        ? current_err_after - current_err_before
	                                        : std::numeric_limits<double>::quiet_NaN();
		        int current_improved = std::isfinite(current_err_delta) ? (current_err_delta < 0.0 ? 1 : 0) : -1;

		        validation_csv_ << ros::Time::now().toSec() << ','
	             << val.self_id << ','
	             << val.target_id << ','
	             << val.obs_type << ','
	             << (val.accepted ? 1 : 0) << ','
	             << (val.replay_ok ? 1 : 0) << ','
	             << (val.found_target_update ? 1 : 0) << ','
	             << val.reason << ','
	             << val.obs_stamp.toSec() << ','
	             << val.delayed_update_stamp.toSec() << ','
	             << val.current_replay_stamp.toSec() << ','
	             << val.best_idx << ','
	             << val.cache_size << ','
	             << delayed_err_before << ','
	             << delayed_err_after << ','
	             << delayed_err_delta << ','
	             << delayed_improved << ','
	             << current_err_before << ','
	             << current_err_after << ','
	             << current_err_delta << ','
	             << current_improved << ','
	             << val.delayed_update_before(0) << ',' << val.delayed_update_before(1) << ',' << val.delayed_update_before(2) << ','
	             << val.delayed_update_after(0) << ',' << val.delayed_update_after(1) << ',' << val.delayed_update_after(2) << ','
	             << val.delayed_update_gt.x() << ',' << val.delayed_update_gt.y() << ',' << val.delayed_update_gt.z() << ','
	             << val.current_replay_before(0) << ',' << val.current_replay_before(1) << ',' << val.current_replay_before(2) << ','
	             << val.current_replay_after(0) << ',' << val.current_replay_after(1) << ',' << val.current_replay_after(2) << ','
	             << val.current_replay_gt.x() << ',' << val.current_replay_gt.y() << ',' << val.current_replay_gt.z() << ','
	             << val.residual_norm << ','
	             << val.nis << ','
	             << val.gain_weight << ','
	             << mission_stage_ << '\n';
	    }

	    void fillDelayedValidationGtAndWrite(DelayedUpdateValidation &val)
	    {
	        if (!val.enabled) return;
	        val.has_delayed_update_gt =
	            getRelativeGroundTruth(val.self_id, val.target_id,
	                                   val.delayed_update_stamp,
	                                   val.delayed_update_gt);
	        val.has_current_replay_gt =
	            getRelativeGroundTruth(val.self_id, val.target_id,
	                                   val.current_replay_stamp,
	                                   val.current_replay_gt);
	        writeDelayedValidationCsv(val);
	    }

	    void writeDelayedAgeSummary()
	    {
	        if (csv_dir_.empty()) return;

	        const std::string self_name = "iris_" + std::to_string(uav_id_);
	        const std::string path = csv_dir_ + "/" + self_name + "_delayed_avg_age.txt";
	        std::ofstream f(path.c_str(), std::ios::out | std::ios::trunc);
	        if (!f.is_open()) return;

	        f << std::fixed << std::setprecision(2);

	        if (delayed_attempt_age_count_ > 0)
	        {
	            const double avg_ms = (delayed_attempt_age_sum_ / delayed_attempt_age_count_) * 1000.0;
	            f << "延迟观测尝试次数：" << delayed_attempt_age_count_ << "\n";
	            f << "延迟观测平均时间延迟（尝试）：" << avg_ms << " ms\n";
	        }
	        else
	        {
	            f << "延迟观测尝试次数：0\n";
	        }

	        if (delayed_accepted_age_count_ > 0)
	        {
	            const double avg_ms = (delayed_accepted_age_sum_ / delayed_accepted_age_count_) * 1000.0;
	            f << "延迟观测接受次数：" << delayed_accepted_age_count_ << "\n";
	            f << "延迟观测平均时间延迟（接受）：" << avg_ms << " ms\n";
	        }
	        else
	        {
	            f << "延迟观测接受次数：0\n";
	        }

	        f.close();
	    }

		UpdateStatus applyDelayedUpdate(Filter &f, const Observation &obs)
	    {
	        const double age = (f.stamp - obs.stamp).toSec();

	        // 统计延迟观测 age
		        if (std::isfinite(age))
		        {
		            delayed_attempt_age_sum_ += age;
		            delayed_attempt_age_count_++;
		        }

	        if (f.cache.empty())
        {
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "empty_history_cache", age,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN());
            return UpdateStatus::REJECTED;
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
            return UpdateStatus::REJECTED;
        }

        //找到了best_idx
	        // 原子化 delayed update：
	        // 在候选 cache 中插入新观测，并从 best_idx 节点开始 replay。
	        // 新 delayed obs 的 hard/NIS gate 在 replay 中、按实际插入顺序执行，
	        // 避免用旧 hist.X_post/P_post 做近似接纳判断。
	        // replay 成功且最终状态/协方差合法后，才提交到真实 filter。

	        // ── delayed update validation ──────────────────────
	        DelayedUpdateValidation val;
	        val.enabled = delayed_validation_enabled_;
	        val.self_id = uav_id_;
	        val.target_id = obs.target_id;
	        val.obs_type = obsTypeName(obs.type);
	        val.obs_stamp = obs.stamp;
	        val.best_idx = best_idx;
	        val.cache_size = static_cast<int>(f.cache.size());

		        if (delayed_validation_enabled_)
		        {
		            val.current_replay_before = f.cache.back().X_post;
		            val.current_replay_stamp = f.cache.back().stamp;
		        }

		        // ── 保存 replay 前当前状态 (供 replay trace 使用) ──
		        const Vector6d current_before_replay = f.cache.back().X_post;
		        const ros::Time current_stamp_before = f.cache.back().stamp;

		        std::deque<History_Record> candidate_cache = f.cache;
		        const uint64_t seq_before = obs_seq_;
		        const uint64_t gated_seq =
		            recordAcceptedObservation(candidate_cache[static_cast<size_t>(best_idx)], obs, true,
		                                      delayed_validation_enabled_);

		        std::string replay_reason;
		        double replay_residual_norm = std::numeric_limits<double>::quiet_NaN();
		        double replay_nis = std::numeric_limits<double>::quiet_NaN();
		        bool failed_on_gated_observation = false;
		        AdaptiveUwbInfo replay_adaptive_uwb;

		        // ── ReplayTraceContext ────────────────────────────────
		        ReplayTraceContext trace_ctx;
		        trace_ctx.enabled = traceActive();
		        trace_ctx.self_id = uav_id_;
		        trace_ctx.target_id = obs.target_id;
		        trace_ctx.stage = mission_stage_;
		        trace_ctx.obs_stamp = obs.stamp;
		        trace_ctx.best_idx = best_idx;
		        trace_ctx.cache_size = static_cast<int>(f.cache.size());
		        trace_ctx.current_stamp = current_stamp_before;

	        if (!replayCacheFrom(candidate_cache, best_idx, true, gated_seq,
	                             replay_reason, replay_residual_norm, replay_nis,
	                             failed_on_gated_observation, replay_adaptive_uwb,
	                             delayed_validation_enabled_ ? &val : nullptr,
	                             traceActive() ? &trace_ctx : nullptr))
        {
            obs_seq_ = seq_before;

            if (failed_on_gated_observation && std::isfinite(replay_residual_norm))
            {
		            const RecoveryResult recovery =
		                tryDgoRecovery(f, obs, true, age, best_dt,
		                               replay_residual_norm, replay_nis, replay_reason);
		            if (recovery == RecoveryResult::RESET)
		            {
		                if (delayed_validation_enabled_)
		                {
		                    val.accepted = false; val.replay_ok = false;
		                    val.reason = "recovery_reset_" + replay_reason;
		                    val.current_replay_after = val.current_replay_before;
		                    fillDelayedValidationGtAndWrite(val);
		                }
		                return UpdateStatus::RECOVERY_RESET;
		            }
		            if (recovery == RecoveryResult::HANDLED_REJECTED)
		            {
		                if (delayed_validation_enabled_)
		                {
		                    val.accepted = false; val.replay_ok = false;
		                    val.reason = "recovery_rejected_" + replay_reason;
		                    val.current_replay_after = val.current_replay_before;
		                    fillDelayedValidationGtAndWrite(val);
		                }
		                return UpdateStatus::REJECTED;
		            }
            }

            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "cache_replay_failed_" + replay_reason, age,
                              best_dt,
                              replay_residual_norm,
                              replay_nis,
                              std::numeric_limits<double>::quiet_NaN(),
	                              replay_adaptive_uwb);
		            if (delayed_validation_enabled_)
		            {
		                val.accepted = false; val.replay_ok = false;
		                val.reason = "cache_replay_failed_" + replay_reason;
		                val.current_replay_after = val.current_replay_before;
		                fillDelayedValidationGtAndWrite(val);
		            }
	            return UpdateStatus::REJECTED;
	        }

	        const History_Record &final_rec = candidate_cache.back();
        if (!final_rec.X_post.allFinite() || !isBoundedCovariance(final_rec.P_post))
        {
            obs_seq_ = seq_before;
            ++f.delayed_p_skips;
            ++f.rejects;
            logObservationRow("delayed_update", f, obs, 0, 1, 0,
                              "cache_replay_rejected_bad_final_cov", age,
                              best_dt,
                              replay_residual_norm,
                              replay_nis,
                              std::numeric_limits<double>::quiet_NaN(),
	                              replay_adaptive_uwb);
	            if (delayed_validation_enabled_)
		            {
		                val.accepted = false; val.replay_ok = true;
		                val.reason = "bad_final_cov";
		                val.current_replay_after = candidate_cache.back().X_post;
		                val.current_replay_stamp = candidate_cache.back().stamp;
		                fillDelayedValidationGtAndWrite(val);
		            }
	            return UpdateStatus::REJECTED;
	        }

	        if (delayed_validation_enabled_)
	        {
	            val.current_replay_after = candidate_cache.back().X_post;
	            val.current_replay_stamp = candidate_cache.back().stamp;
	        }

		        f.cache = std::move(candidate_cache);
		        f.X = f.cache.back().X_post;
		        f.P = f.cache.back().P_post;
		        symmetrizeCov(f.P);
		        incrementAcceptedCounters(f, obs, true);

		        if (delayed_validation_enabled_)
		        {
		            val.accepted = true; val.replay_ok = true;
		            val.reason = "accepted";
		            fillDelayedValidationGtAndWrite(val);
		        }

	        // ── write replay trace ─────────────────────────────
		        if (traceActive() && trace_ctx.found_target_update)
		        {
		            const Vector6d &current_before = current_before_replay;
		            const Vector6d &current_after = f.cache.back().X_post;

		            const int replay_nodes = static_cast<int>(f.cache.size()) - 1 - best_idx;
		            const double replay_duration = current_stamp_before.isZero()
		                ? 0.0 : (current_stamp_before - trace_ctx.hist_stamp).toSec();

		            writeReplayTrace(
		                f, obs, trace_ctx,
		                current_before, current_after,
		                std::max(replay_nodes, 0), std::abs(replay_duration));
		        }

		logObservationRow("delayed_update", f, obs, 1, 1, 1,
	                          "accepted_replayed", age,
                          best_dt,
                          replay_residual_norm,
                          replay_nis,
                          std::numeric_limits<double>::quiet_NaN(),
	                          replay_adaptive_uwb);

	        // 统计被接受的 delayed update age
	        if (std::isfinite(age))
	        {
	            delayed_accepted_age_sum_ += age;
	            delayed_accepted_age_count_++;
	        }

	        return UpdateStatus::ACCEPTED;
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

	        // state trace (每 publish 一次记录)
	        if (traceActive())
	            writeStateTrace(f);
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
