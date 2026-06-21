#include <ros/ros.h>
#include <ros/package.h>
#include <sensors/ComMsg.h>
#include <data_process/CameraAngleMatch.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <data_process/UwbProcessed.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8.h>
#include <LBFGS.h>

#define EKF_X_STDDEV 0.845622420
#define EKF_Y_STDDEV 0.841162516
#define EKF_Z_STDDEV 0.080471782
#define ANGLE_ALPHA_STDDEV 0.02725
#define ANGLE_THETA_STDDEV 0.0379
class DGO
{
public:
    DGO() : nh_(), pnh_("~")
    {
        pnh_.param<int>("uav_id",uav_id_,0);
        pnh_.param<int>("uav_num",uav_num_,4);
        pnh_.param("initial_spacing", initial_spacing_, 2.0);
        pnh_.param("max_sensor_age", max_sensor_age_, 0.5);
        pnh_.param("use_gazebo_initial_offsets", use_gazebo_initial_offsets_, true);
        pnh_.param("origin_capture_delay", origin_capture_delay_, 1.0);
        pnh_.param("oracle_mode", oracle_mode_, false);
        pnh_.param("max_communication_age", max_communication_age_, 0.5);
        pnh_.param("max_communication_skew", max_communication_skew_, 0.2);
        pnh_.param("max_gt_age", max_gt_age_, 0.1);
        pnh_.param("max_extrapolation_dt", max_extrapolation_dt_, 0.2);
        pnh_.param("uwb_stddev", uwb_stddev_, 0.05);
        pnh_.param("uwb_dynamic_stddev", uwb_dynamic_stddev_, 0.065);
        pnh_.param("ins_prior_stddev_xy", ins_prior_stddev_xy_, 0.25);
        pnh_.param("max_dgo_correction_xy", max_dgo_correction_xy_, 0.20);
        pnh_.param("max_dgo_step_xy", max_dgo_step_xy_, 0.30);
        pnh_.param("require_airborne_initialization",
                   require_airborne_initialization_, true);
        pnh_.param("min_dgo_start_altitude",
                   min_dgo_start_altitude_, 0.35);
        if (min_dgo_start_altitude_ < 0.0 ||
            uwb_stddev_ <= 0.0 ||
            uwb_dynamic_stddev_ <= 0.0 ||
            ins_prior_stddev_xy_ <= 0.0 ||
            max_dgo_correction_xy_ <= 0.0 ||
            max_dgo_step_xy_ <= 0.0)
        {
            ROS_FATAL("[DGO] invalid parameter: min_start_altitude=%.3f "
                      "uwb_stddev=%.3f uwb_dynamic_stddev=%.3f "
                      "ins_prior_stddev_xy=%.3f max_correction_xy=%.3f "
                      "max_step_xy=%.3f",
                      min_dgo_start_altitude_, uwb_stddev_,
                      uwb_dynamic_stddev_, ins_prior_stddev_xy_,
                      max_dgo_correction_xy_, max_dgo_step_xy_);
            ros::shutdown();
            return;
        }

        Pc.resize(uav_num_);
        com_positions_.resize(uav_num_);
        com_velocities_.resize(uav_num_);
        has_com_.resize(uav_num_,false);
        has_new_com_.resize(uav_num_,false);
        last_seq_.resize(uav_num_,UINT32_MAX);
        data_timestamps_.resize(uav_num_);
        dist_error_.resize(uav_num_,0.0);
        dist_xy_error_.resize(uav_num_);
        dist_xy_error_valid_.resize(uav_num_, false);
        angle_err_.resize(uav_num_);
        angle_error_valid_.resize(uav_num_, false);
        camera_target_valid_.resize(uav_num_, false);
        gt_positions_.resize(uav_num_);
        has_gt_.resize(uav_num_, false);
        best_gt_positions_.resize(uav_num_);
        best_gt_valid_.resize(uav_num_, false);
        origins_.resize(uav_num_, Eigen::Vector3d::Zero());
        origin_received_.resize(uav_num_, false);
        if (uav_num_ > static_cast<int>(uavs_.size()) || uav_id_ < 0 || uav_id_ >= uav_num_)
        {
            ROS_FATAL("[DGO] invalid uav_id=%d uav_num=%d, supported uav_num<=%zu",
                      uav_id_, uav_num_, uavs_.size());
            ros::shutdown();
            return;
        }
        initInitialOffsets();
        
        //对于本机id，订阅其他所有无人机的通信节点
        for(int i=0;i<uav_num_;i++)
        {
            if(i == uav_id_)
            {
                continue;
            }
            std::string com_topic = uavs_[i] + "/communication";
            com_subs_.push_back(nh_.subscribe<sensors::ComMsg>(
                com_topic,
                20,
                boost::bind(&DGO::comCallback,this,_1,i)));
        }

        //订阅本机的INS估计
        std::string ins_topic_ = uavs_[uav_id_] + "/ins_estimate";
        ins_sub_ = nh_.subscribe(ins_topic_,10,&DGO::insCallback,this);

        //DGO
        DGO_Timer_ = nh_.createTimer(
            ros::Duration(0.1),
            &DGO::dgoTimerCallback,
            this
        );

        //获取UWB测距结果
        std::string uwb_topic = uavs_[uav_id_] + "/uwb_processed";
        uwb_sub_ = nh_.subscribe(uwb_topic, 10, &DGO::uwbCallback, this);

        //获取相机测量角度结果
        std::string camera_topic = uavs_[uav_id_] + "/camera_angle_match";
        camera_sub_ = nh_.subscribe(camera_topic,10,&DGO::cameraCallback,this);

        model_sub_ = nh_.subscribe("/gazebo/model_states", 10, &DGO::modelStatesCallback, this);
        stage_sub_ = nh_.subscribe("/formation/stage", 1, &DGO::stageCallback, this);

        dgo_pub_ = nh_.advertise<nav_msgs::Odometry>("dgo_estimate", 10);
        initResidualCsv();
        initCommDebugCsv();
        initSyncDiagCsv();

        ROS_INFO("[DGO] ns=%s uav_id=%d uav_num=%d initial_spacing=%.2f "
                 "max_sensor_age=%.2f max_communication_age=%.2f "
                 "max_communication_skew=%.2f max_gt_age=%.2f "
                 "optimization=XY z_source=INS "
                 "uwb_stddev=%.3f dynamic=%.3f "
                 "ins_prior_stddev_xy=%.2f max_correction_xy=%.2f "
                 "max_step_xy=%.2f "
                 "airborne_init=%d min_start_altitude=%.2f oracle_mode=%d",
                 ros::this_node::getNamespace().c_str(), uav_id_, uav_num_,
                 initial_spacing_, max_sensor_age_, max_communication_age_,
                 max_communication_skew_, max_gt_age_,
                 uwb_stddev_, uwb_dynamic_stddev_,
                 ins_prior_stddev_xy_, max_dgo_correction_xy_,
                 max_dgo_step_xy_,
                 require_airborne_initialization_ ? 1 : 0,
                 min_dgo_start_altitude_,
                 oracle_mode_ ? 1 : 0);
    }

    ~DGO()
    {
        if (residual_csv_.is_open())
            residual_csv_.close();
        if (comm_debug_csv_.is_open())
            comm_debug_csv_.close();
        if (sync_diag_csv_.is_open())
            sync_diag_csv_.close();
    }

    void comCallback(const sensors::ComMsg::ConstPtr &msg, int uav_index)
    {
        com_positions_[uav_index] = msg->position;
        com_velocities_[uav_index] = msg->velocity;
        Pc[uav_index] = msg->position;
        data_timestamps_[uav_index] = msg->header.stamp;

        if (!has_com_[uav_index])
        {
            has_com_[uav_index] = true;
            has_new_com_[uav_index] = true;
            last_seq_[uav_index] = msg->header.seq;
            return;
        }

        if (msg->header.seq != last_seq_[uav_index])
        {
            last_seq_[uav_index] = msg->header.seq;
            has_new_com_[uav_index] = true;
        }
        else
        {
            has_new_com_[uav_index] = false;
        }
    }
    void insCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        latest_ins_msg_ = msg;

        geometry_msgs::Point P_i;
        P_i.x = msg->pose.pose.position.x;
        P_i.y = msg->pose.pose.position.y;
        P_i.z = msg->pose.pose.position.z;
        Pc[uav_id_] = P_i;
        data_timestamps_[uav_id_] = msg->header.stamp;

        if(!has_com_[uav_id_])
        {
            P_opt_.x = P_i.x;
            P_opt_.y = P_i.y;
            P_opt_.z = P_i.z;
        }

        // 缓存 INS 位置, 用于后续按 sync_ref_time_ 统一挑选
        ins_history_.push_back({msg->header.stamp, P_i});
        if (ins_history_.size() > 20)
            ins_history_.pop_front();

        // 标记本地 INS 数据已收到 (不再依赖其他机通信是否到齐)
        has_com_[uav_id_] = true;
        has_new_com_[uav_id_] = (msg->header.seq != last_seq_[uav_id_]);
        last_seq_[uav_id_] = msg->header.seq;
    }

    void uwbCallback(const data_process::UwbProcessed::ConstPtr &msg)
    {
        // 仅缓存, 具体挑选在 isReadyForDGO() 中按 sync_ref_time_ 统一进行
        uwb_history_.push_back({msg->header.stamp, *msg});
        if (uwb_history_.size() > 20)
            uwb_history_.pop_front();
        has_uwb_ = true;
    }

    void cameraCallback(const data_process::CameraAngleMatch::ConstPtr &msg)
    {
        // 仅缓存, 具体挑选在 isReadyForDGO() 中按 sync_ref_time_ 统一进行
        camera_history_.push_back({msg->header.stamp, *msg});
        if (camera_history_.size() > 20)
            camera_history_.pop_front();
        has_camera_ = true;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        const ros::Time sample_stamp = ros::Time::now();

        // — 首次收到 model_states 时锁定各机初始位置 (论文初始编队偏移) —
        if (use_gazebo_initial_offsets_ && !origin_locked_)
        {
            const ros::Time now = sample_stamp;
            if (origin_capture_start_.isZero())
                origin_capture_start_ = now;

            for (int i = 0; i < uav_num_; ++i)
            {
                if (origin_received_[i])
                    continue;
                auto it = std::find(msg->name.begin(), msg->name.end(),
                                    uavs_[i].substr(1));
                if (it == msg->name.end())
                    continue;
                const size_t k = std::distance(msg->name.begin(), it);
                if (k >= msg->pose.size())
                    continue;
                origins_[i].x() = msg->pose[k].position.x;
                origins_[i].y() = msg->pose[k].position.y;
                origins_[i].z() = msg->pose[k].position.z;
                origin_received_[i] = true;
            }

            bool all_received = true;
            for (int i = 0; i < uav_num_; ++i)
                all_received = all_received && origin_received_[i];

            if (all_received &&
                (now - origin_capture_start_).toSec() >= origin_capture_delay_)
            {
                // 以 iris_0 为参考原点 (与论文一致)
                for (int i = 0; i < uav_num_; ++i)
                {
                    initial_offsets_[i] = origins_[i] - origins_[0];
                }
                origin_locked_ = true;
                ROS_INFO("[iris_%d] DGO initial offsets locked from Gazebo after %.2fs: "
                         "iris_0=(%.2f,%.2f,%.2f) iris_1=(%.2f,%.2f,%.2f) "
                         "iris_2=(%.2f,%.2f,%.2f) iris_3=(%.2f,%.2f,%.2f)",
                         uav_id_,
                         (now - origin_capture_start_).toSec(),
                         origins_[0].x(), origins_[0].y(), origins_[0].z(),
                         origins_[1].x(), origins_[1].y(), origins_[1].z(),
                         origins_[2].x(), origins_[2].y(), origins_[2].z(),
                         origins_[3].x(), origins_[3].y(), origins_[3].z());
            }
        }

        // 持续缓存带仿真时间戳的 GT 快照，Oracle 和 debug 都按
        // sync_ref_time_ 选择最近快照，避免动态阶段混用 now 与历史测量。
        GtSample sample;
        sample.stamp = sample_stamp;
        sample.positions.resize(uav_num_);
        sample.valid.resize(uav_num_, false);
        for (int i = 0; i < uav_num_; ++i)
        {
            const std::string model_name = uavs_[i].substr(1);
            auto it = std::find(msg->name.begin(), msg->name.end(), model_name);
            if (it == msg->name.end())
                continue;

            const size_t idx = std::distance(msg->name.begin(), it);
            if (idx >= msg->pose.size())
                continue;

            gt_positions_[i] = msg->pose[idx].position;
            has_gt_[i] = true;
            sample.positions[i] = msg->pose[idx].position;
            sample.valid[i] = true;
        }
        gt_history_.push_back(sample);
        while (gt_history_.size() > 300)
            gt_history_.pop_front();
    }

    void stageCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        mission_stage_ = msg->data;
    }

    void dgoTimerCallback(const ros::TimerEvent& event)
    {
        if(isReadyForDGO())
        {
            //DGO算法
            RunDGO();

            // 重置新数据标记, 等待下一轮所有数据更新后再触发
            std::fill(has_new_com_.begin(), has_new_com_.end(), false);
        }
    }

    void RunDGO()
    {
        // 保留上一轮 DGO 优化的 P_opt_ 结果
        prev_P_opt_ = P_opt_;
        ins_delta_valid_ = prepareInsDelta();

        geometry_msgs::Point predicted = prev_P_opt_;
        if (ins_delta_valid_)
        {
            predicted.x += ins_delta_.x;
            predicted.y += ins_delta_.y;
            predicted.z += ins_delta_.z;
        }

        // DGO 只优化水平位置。垂直状态完全继承 INS 短时增量，
        // 避免 UWB 球面约束和偶发 theta 异常把 Z 拉偏。
        P_opt_ = predicted;
        Eigen::VectorXd x(2);
        x << predicted.x, predicted.y;

        LBFGSpp::LBFGSParam<double> param;
        param.epsilon = 1e-5;
        param.max_iterations = 20;
        param.m = 6;

        LBFGSpp::LBFGSSolver<double> solver(param);

        DGOFunctor functor(this);

        double final_cost = 0.0;

        try
        {
            CostBreakdown before_cb = computeCostBreakdown();
            double before_cost = before_cb.total();
            writeResidualDebugCsv("pre", before_cb.ins);

            int niter = solver.minimize(functor, x, final_cost);

            geometry_msgs::Point candidate;
            candidate.x = x[0];
            candidate.y = x[1];
            if (!std::isfinite(candidate.x) ||
                !std::isfinite(candidate.y))
            {
                throw std::runtime_error("non-finite optimizer result");
            }

            const double correction_x = candidate.x - predicted.x;
            const double correction_y = candidate.y - predicted.y;
            const double correction_norm =
                std::sqrt(correction_x * correction_x +
                          correction_y * correction_y);
            double correction_scale = 1.0;
            if (correction_norm > max_dgo_correction_xy_ &&
                correction_norm > 1e-9)
            {
                correction_scale = max_dgo_correction_xy_ / correction_norm;
            }

            P_opt_.x = predicted.x + correction_x * correction_scale;
            P_opt_.y = predicted.y + correction_y * correction_scale;
            P_opt_.z = predicted.z;

            const double step_x = P_opt_.x - prev_P_opt_.x;
            const double step_y = P_opt_.y - prev_P_opt_.y;
            const double step_norm =
                std::sqrt(step_x * step_x + step_y * step_y);
            if (step_norm > max_dgo_step_xy_ && step_norm > 1e-9)
            {
                const double scale = max_dgo_step_xy_ / step_norm;
                P_opt_.x = prev_P_opt_.x + step_x * scale;
                P_opt_.y = prev_P_opt_.y + step_y * scale;
            }
            if (correction_scale < 1.0)
            {
                ROS_WARN_THROTTLE(
                    5.0,
                    "[iris_%d] DGO XY correction limited: raw=(%.3f %.3f) "
                    "limit_xy=%.2f",
                    uav_id_, correction_x, correction_y,
                    max_dgo_correction_xy_);
            }
            commitInsDelta();

            CostBreakdown after_cb = computeCostBreakdown();
            updateCameraCounts();
            writeResidualDebugCsv("post", after_cb.ins);
            writeSyncDiagCsv();
            writeCommDebugCsv();
            publishDgoEstimate();

            ROS_INFO_THROTTLE(5.0,
                "[iris_%d] cost: %.2f -> %.2f (%.1f%%), iter=%d | "
                "uwb=%.2f/%.2f angle=%.2f/%.2f xy=%.2f/%.2f ins=%.2f/%.2f",
                uav_id_,
                before_cost, final_cost,
                (1.0 - final_cost / before_cost) * 100.0,
                niter,
                before_cb.dist, after_cb.dist,
                before_cb.angle, after_cb.angle,
                before_cb.xy, after_cb.xy,
                before_cb.ins, after_cb.ins);

            ROS_INFO_THROTTLE(5.0,
                "[iris_%d] camera: fresh=%d raw=%d valid=%d "
                "angle_constraints=%d xy_constraints=%d used=%d age=%s",
                uav_id_,
                camera_message_fresh_ ? 1 : 0,
                camera_raw_count_,
                camera_valid_target_count_,
                camera_angle_constraint_count_,
                camera_xy_constraint_count_,
                camera_used_in_cost_ ? 1 : 0,
                cameraAgeStr().c_str());

            if (camera_message_fresh_ && !camera_used_in_cost_)
            {
                ROS_WARN_THROTTLE(5.0,
                    "[iris_%d] fresh camera message produced no valid constraints: "
                    "count=%u ids=%zu alpha=%zu theta=%zu",
                    uav_id_,
                    best_camera_.count,
                    best_camera_.id.size(),
                    best_camera_.alpha.size(),
                    best_camera_.theta.size());
            }
        }
        catch(const std::exception& e)
        {
            if (ins_delta_valid_)
            {
                P_opt_.x = prev_P_opt_.x + ins_delta_.x;
                P_opt_.y = prev_P_opt_.y + ins_delta_.y;
                P_opt_.z = prev_P_opt_.z + ins_delta_.z;
                commitInsDelta();
                publishDgoEstimate();
                ROS_WARN("[iris_%d] L-BFGS failed: %s; fallback to INS delta prediction",
                         uav_id_, e.what());
            }
            else
            {
                ROS_WARN("[iris_%d] L-BFGS failed: %s", uav_id_, e.what());
            }
        }
    }

private:
    int uav_id_;
    int uav_num_;
    double initial_spacing_ = 2.0;
    double max_sensor_age_ = 0.5;
    double max_communication_age_ = 0.5;
    double max_communication_skew_ = 0.2;
    double max_gt_age_ = 0.1;
    double max_extrapolation_dt_ = 0.2;
    double uwb_stddev_ = 0.05;
    double uwb_dynamic_stddev_ = 0.065;
    double ins_prior_stddev_xy_ = 0.25;
    double max_dgo_correction_xy_ = 0.20;
    double max_dgo_step_xy_ = 0.30;
    double min_dgo_start_altitude_ = 0.35;
    bool require_airborne_initialization_ = true;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::vector<ros::Subscriber> com_subs_;
    ros::Subscriber ins_sub_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber model_sub_;
    ros::Subscriber stage_sub_;
    ros::Publisher dgo_pub_;

    std::vector<std::string> uavs_ = {"/iris_0","/iris_1","/iris_2","/iris_3"};
    std::vector<geometry_msgs::Point> Pc;
    std::vector<geometry_msgs::Point> com_positions_;
    std::vector<geometry_msgs::Vector3> com_velocities_;
    std::vector<Eigen::Vector3d> initial_offsets_;
    std::vector<bool> has_new_com_;
    std::vector<bool> has_com_;
    std::vector<uint32_t> last_seq_;
    std::vector<ros::Time> data_timestamps_;   // 各无人机数据更新时间戳

    // Gazebo GT 缓存, 用于 DGO residual 调试输出
    std::vector<geometry_msgs::Point> gt_positions_;
    std::vector<bool> has_gt_;
    struct GtSample
    {
        ros::Time stamp;
        std::vector<geometry_msgs::Point> positions;
        std::vector<bool> valid;
    };
    std::deque<GtSample> gt_history_;
    std::vector<geometry_msgs::Point> best_gt_positions_;
    std::vector<bool> best_gt_valid_;
    ros::Time best_gt_stamp_;

    // residual debug CSV
    std::string csv_dir_;
    std::ofstream residual_csv_;

    // communication vs GT debug CSV
    std::ofstream comm_debug_csv_;
    // 同步诊断 CSV (每轮 DGO 记录各传感器年龄)
    std::ofstream sync_diag_csv_;

    // UWB 缓存
    std::deque<std::pair<ros::Time, data_process::UwbProcessed>> uwb_history_;
    data_process::UwbProcessed best_uwb_;
    ros::Time best_uwb_stamp_;
    bool has_uwb_ = false;

    // CameraAngleMatch 缓存
    std::deque<std::pair<ros::Time, data_process::CameraAngleMatch>> camera_history_;
    data_process::CameraAngleMatch best_camera_;
    ros::Time best_camera_stamp_;
    bool has_camera_ = false;

    

    // INS 缓存
    std::deque<std::pair<ros::Time, geometry_msgs::Point>> ins_history_;
    nav_msgs::Odometry::ConstPtr latest_ins_msg_;
    geometry_msgs::Point best_ins_pos_;
    ros::Time best_ins_stamp_;
    geometry_msgs::Point prev_ins_pos_;
    geometry_msgs::Point ins_delta_;    // 相邻 DGO 轮次间本机位移
    bool has_ins_ = false;
    bool ins_update_ = false;
    bool ins_delta_valid_ = false;
    bool has_prev_ins_pos_ = false;

    //DGO
    ros::Timer DGO_Timer_;
    geometry_msgs::Point P_opt_;
    geometry_msgs::Point prev_P_opt_;
    std::vector<double> dist_error_;    //距离误差
    struct AngleError
    {
        double alpha = 0.0;
        double theta = 0.0;
    };
    std::vector<AngleError> angle_err_; //角度误差, 按无人机 ID 索引
    struct DistXYError
    {
        double x = 0.0;
        double y = 0.0;
        double sigma_x2 = 0.05 * 0.05;
        double sigma_y2 = 0.05 * 0.05;
    };
    std::vector<DistXYError> dist_xy_error_; //UWB-相机距离误差, 按无人机 ID 索引
    std::vector<bool> dist_xy_error_valid_;
    std::vector<bool> angle_error_valid_;  // 逐 target 标记角度误差是否有效
    std::vector<Eigen::Vector3d> origins_;
    std::vector<bool> origin_received_;
    bool origin_locked_ = false;
    ros::Time origin_capture_start_;
    double origin_capture_delay_ = 1.0;
    bool use_gazebo_initial_offsets_ = true;
    bool oracle_mode_ = false;
    ros::Time sync_ref_time_;
    // 相机约束状态拆分 (替代旧 use_camera_in_cost_)
    bool camera_message_fresh_ = false;
    int camera_raw_count_ = 0;           // 相机消息中 count 字段原值
    int camera_valid_target_count_ = 0;  // ID合法、角度有限、非本机
    int camera_angle_constraint_count_ = 0;  // 实际进入 Jθ 的目标数
    int camera_xy_constraint_count_ = 0;     // 同时匹配 UWB 的 Jrp 目标数
    bool camera_used_in_cost_ = false;   // 至少一个约束进入总代价
    std::vector<bool> camera_target_valid_;  // 统一供角度/XY 使用
    uint8_t mission_stage_ = 0;
    bool dgo_started_ = false;

    struct CostBreakdown
    {
        double angle = 0.0;
        double dist  = 0.0;
        double xy    = 0.0;
        double ins   = 0.0;
        double total() const { return angle + dist + xy + ins; }
    };

    std::string defaultCsvDir() const
    {
        const std::string test_pkg = ros::package::getPath("test");
        if (!test_pkg.empty())
            return test_pkg + "/logs";
        return "/tmp";
    }

    bool ensureDirectory(const std::string &dir) const
    {
        boost::system::error_code ec;
        boost::filesystem::create_directories(dir, ec);
        return !ec;
    }

    void initResidualCsv()
    {
        pnh_.param<std::string>("csv_dir", csv_dir_, defaultCsvDir());
        if (!ensureDirectory(csv_dir_))
        {
            ROS_WARN("[iris_%d] cannot create residual debug csv dir: %s",
                     uav_id_, csv_dir_.c_str());
            return;
        }

        const std::string self_name = uavs_[uav_id_].substr(1);
        const std::string path = csv_dir_ + "/" + self_name + "_dgo_residual_debug.csv";
        residual_csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!residual_csv_.is_open())
        {
            ROS_WARN("[iris_%d] cannot open residual debug csv: %s",
                     uav_id_, path.c_str());
            return;
        }

        residual_csv_ << "stamp,phase,self_id,target_id,"
                      << "uwb_dt,camera_dt,com_dt,"
                      << "rel_gt_x,rel_gt_y,rel_gt_z,"
                      << "rel_pred_x,rel_pred_y,rel_pred_z,"
                      << "uwb_meas,uwb_pred,uwb_residual,"
                      << "alpha_meas,alpha_pred,alpha_residual,"
                      << "theta_meas,theta_pred,theta_residual,"
                      << "xy_residual_x,xy_residual_y,"
                      << "cost_uwb,cost_angle,cost_xy,cost_ins,"
                      << "cam_target_valid,angle_constraint_valid,"
                      << "xy_constraint_valid\n";
        residual_csv_ << std::fixed << std::setprecision(9);
        ROS_INFO("[iris_%d] residual debug CSV: %s", uav_id_, path.c_str());
    }

    void initCommDebugCsv()
    {
        if (csv_dir_.empty())
            return;

        const std::string self_name = uavs_[uav_id_].substr(1);
        const std::string path = csv_dir_ + "/" + self_name + "_comm_debug.csv";
        comm_debug_csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!comm_debug_csv_.is_open())
        {
            ROS_WARN("[iris_%d] cannot open comm debug csv: %s", uav_id_, path.c_str());
            return;
        }

        comm_debug_csv_ << "stamp,self_id,target_id,"
                        << "com_global_x,com_global_y,com_global_z,"
                        << "gt_x,gt_y,gt_z,"
                        << "com_error_x,com_error_y,com_error_z,"
                        << "com_error_norm,"
                        << "com_stamp_dt\n";
        comm_debug_csv_ << std::fixed << std::setprecision(9);
        ROS_INFO("[iris_%d] comm debug CSV: %s", uav_id_, path.c_str());
    }

    void initSyncDiagCsv()
    {
        if (csv_dir_.empty())
            return;
        const std::string self_name = uavs_[uav_id_].substr(1);
        const std::string sync_dir = csv_dir_ + "/sensor_sync_logs";
        ensureDirectory(sync_dir);
        const std::string path = sync_dir + "/" + self_name + "_dgo_sync_diag.csv";
        sync_diag_csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!sync_diag_csv_.is_open())
        {
            ROS_WARN("[iris_%d] cannot open sync diag csv: %s", uav_id_, path.c_str());
            return;
        }

        sync_diag_csv_ << "stamp,oracle";
        for (int i = 0; i < uav_num_; ++i)
            if (i != uav_id_)
                sync_diag_csv_ << ",com_dt_" << i << ",com_age_" << i;
        sync_diag_csv_ << ",uwb_dt,uwb_age,camera_dt,camera_age,"
                       << "ins_dt,ins_age,gt_dt,gt_age";
        sync_diag_csv_ << ",mission_stage,uwb_stddev,"
                       << "camera_msg_fresh,camera_raw_count,"
                       << "camera_valid_target,camera_angle_constraints,"
                       << "camera_xy_constraints,camera_used_in_cost,"
                       << "camera_target_mask,"
                       << "P_opt_x,P_opt_y,P_opt_z\n";
        sync_diag_csv_ << std::fixed << std::setprecision(9);
        ROS_INFO("[iris_%d] sync diag CSV: %s", uav_id_, path.c_str());
    }

    void writeSyncDiagCsv()
    {
        if (!sync_diag_csv_.is_open())
            return;

        const double ref = sync_ref_time_.isZero() ? ros::Time::now().toSec()
                                                    : sync_ref_time_.toSec();
        sync_diag_csv_ << ref << ',' << (oracle_mode_ ? 1 : 0);

        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_)
                continue;
            sync_diag_csv_ << ',';
            if (!data_timestamps_[i].isZero())
            {
                const double dt = data_timestamps_[i].toSec() - ref;
                sync_diag_csv_ << dt << ',' << std::fabs(dt);
            }
            else
                sync_diag_csv_ << "nan,nan";
        }

        writeTimeOffset(sync_diag_csv_, best_uwb_stamp_, ref);
        writeTimeOffset(sync_diag_csv_, best_camera_stamp_, ref);
        writeTimeOffset(sync_diag_csv_, best_ins_stamp_, ref);
        writeTimeOffset(sync_diag_csv_, best_gt_stamp_, ref);

        sync_diag_csv_ << ',' << static_cast<unsigned int>(mission_stage_)
                       << ',' << currentUwbStddev()
                       << ',' << (camera_message_fresh_ ? 1 : 0)
                       << ',' << camera_raw_count_
                       << ',' << camera_valid_target_count_
                       << ',' << camera_angle_constraint_count_
                       << ',' << camera_xy_constraint_count_
                       << ',' << (camera_used_in_cost_ ? 1 : 0)
                       << ',' << cameraTargetMask()
                       << ',' << P_opt_.x
                       << ',' << P_opt_.y
                       << ',' << P_opt_.z
                       << '\n';
        sync_diag_csv_.flush();
    }

    // 统一相机目标验证: 成功返回 true, 填 target_id
    bool isValidCameraTarget(size_t index, int &target_id) const
    {
        const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                   best_camera_.id.size(),
                                   best_camera_.alpha.size(),
                                   best_camera_.theta.size()});
        if (index >= n)
            return false;

        target_id = static_cast<int>(best_camera_.id[index]);
        if (target_id < 0 || target_id >= uav_num_ || target_id == uav_id_)
            return false;
        if (!std::isfinite(best_camera_.alpha[index]) ||
            !std::isfinite(best_camera_.theta[index]))
            return false;
        if (best_camera_.alpha[index] < -M_PI ||
            best_camera_.alpha[index] > M_PI)
            return false;
        if (best_camera_.theta[index] < -M_PI_2 ||
            best_camera_.theta[index] > M_PI_2)
            return false;
        return true;
    }

    // 更新每轮相机约束计数 (从 valid 数组统计, 避免数值梯度重复累加)
    void updateCameraCounts()
    {
        camera_valid_target_count_ = 0;
        for (int i = 0; i < uav_num_; ++i)
            if (i != uav_id_ && camera_target_valid_[i])
                ++camera_valid_target_count_;

        camera_angle_constraint_count_ = static_cast<int>(
            std::count(angle_error_valid_.begin(), angle_error_valid_.end(), true));

        camera_xy_constraint_count_ = static_cast<int>(
            std::count(dist_xy_error_valid_.begin(), dist_xy_error_valid_.end(), true));

        camera_used_in_cost_ =
            camera_angle_constraint_count_ > 0 ||
            camera_xy_constraint_count_ > 0;
    }

    // 相机目标掩码 (bit i=1 表示该 target 有至少一条有效约束)
    unsigned cameraTargetMask() const
    {
        unsigned mask = 0;
        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_)
                continue;
            if (angle_error_valid_[i] || dist_xy_error_valid_[i])
                mask |= (1u << i);
        }
        return mask;
    }

    std::string cameraAgeStr() const
    {
        if (best_camera_stamp_.isZero() || sync_ref_time_.isZero())
            return "nan";
        return std::to_string((sync_ref_time_ - best_camera_stamp_).toSec());
    }

    static void writeTimeOffset(std::ofstream &stream,
                                const ros::Time &stamp,
                                double ref)
    {
        stream << ',';
        if (stamp.isZero())
        {
            stream << "nan,nan";
            return;
        }
        const double dt = stamp.toSec() - ref;
        stream << dt << ',' << std::fabs(dt);
    }

    void writeCommDebugCsv()
    {
        if (!comm_debug_csv_.is_open())
            return;

        const double stamp = sync_ref_time_.isZero() ? ros::Time::now().toSec()
                                                     : sync_ref_time_.toSec();

        for (int target_id = 0; target_id < uav_num_; ++target_id)
        {
            if (target_id == uav_id_)
                continue;

            const double com_dt = (!data_timestamps_[target_id].isZero() && !sync_ref_time_.isZero())
                                      ? (data_timestamps_[target_id] - sync_ref_time_).toSec()
                                      : nanValue();

            double com_error_x = nanValue();
            double com_error_y = nanValue();
            double com_error_z = nanValue();
            double com_error_norm = nanValue();

            if (best_gt_valid_[target_id])
            {
                const Eigen::Vector3d target_origin =
                    origin_locked_ ? origins_[target_id] : initial_offsets_[target_id];
                const Eigen::Vector3d target_global(
                    target_origin.x() + Pc[target_id].x,
                    target_origin.y() + Pc[target_id].y,
                    target_origin.z() + Pc[target_id].z);
                com_error_x = target_global.x() - best_gt_positions_[target_id].x;
                com_error_y = target_global.y() - best_gt_positions_[target_id].y;
                com_error_z = target_global.z() - best_gt_positions_[target_id].z;
                com_error_norm = std::sqrt(com_error_x * com_error_x +
                                          com_error_y * com_error_y +
                                          com_error_z * com_error_z);
            }

            const Eigen::Vector3d target_origin =
                origin_locked_ ? origins_[target_id] : initial_offsets_[target_id];
            const Eigen::Vector3d target_global(
                target_origin.x() + Pc[target_id].x,
                target_origin.y() + Pc[target_id].y,
                target_origin.z() + Pc[target_id].z);
            comm_debug_csv_ << stamp << ','
                            << uav_id_ << ','
                            << target_id << ','
                            << target_global.x() << ','
                            << target_global.y() << ','
                            << target_global.z() << ','
                            << (best_gt_valid_[target_id] ? best_gt_positions_[target_id].x : nanValue()) << ','
                            << (best_gt_valid_[target_id] ? best_gt_positions_[target_id].y : nanValue()) << ','
                            << (best_gt_valid_[target_id] ? best_gt_positions_[target_id].z : nanValue()) << ','
                            << com_error_x << ','
                            << com_error_y << ','
                            << com_error_z << ','
                            << com_error_norm << ','
                            << com_dt << '\n';
        }
        comm_debug_csv_.flush();
    }

    static double nanValue()
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    static double sqr(double v)
    {
        return v * v;
    }

    double currentUwbStddev() const
    {
        // Formation Stage::EXPAND_SHRINK == 3. 该阶段距离变化最快，
        // UWB 滑窗会产生额外动态误差，因此使用实测的 0.065m。
        return mission_stage_ == 3 ? uwb_dynamic_stddev_ : uwb_stddev_;
    }

    int findUwbIndexForTarget(int target_id) const
    {
        const size_t n = std::min(best_uwb_.target_ids.size(), best_uwb_.distances.size());
        for (size_t i = 0; i < n; ++i)
        {
            if (best_uwb_.target_ids[i] == target_id)
                return static_cast<int>(i);
        }
        return -1;
    }

    int findCameraIndexForTarget(int target_id) const
    {
        if (!camera_message_fresh_)
            return -1;

        const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                   best_camera_.id.size(),
                                   best_camera_.alpha.size(),
                                   best_camera_.theta.size()});
        for (size_t i = 0; i < n; ++i)
        {
            if (best_camera_.id[i] == target_id)
                return static_cast<int>(i);
        }
        return -1;
    }

    void writeResidualDebugCsv(const std::string &phase, double cost_ins)
    {
        if (!residual_csv_.is_open())
            return;

        const double stamp = sync_ref_time_.isZero() ? ros::Time::now().toSec()
                                                     : sync_ref_time_.toSec();
        const double uwb_dt = (!best_uwb_stamp_.isZero() && !sync_ref_time_.isZero())
                                  ? (best_uwb_stamp_ - sync_ref_time_).toSec()
                                  : nanValue();
        const double camera_dt = (camera_message_fresh_ && !best_camera_stamp_.isZero() && !sync_ref_time_.isZero())
                                     ? (best_camera_stamp_ - sync_ref_time_).toSec()
                                     : nanValue();
        for (int target_id = 0; target_id < uav_num_; ++target_id)
        {
            if (target_id == uav_id_)
                continue;

            const double com_dt = (!data_timestamps_[target_id].isZero() && !sync_ref_time_.isZero())
                                      ? (data_timestamps_[target_id] - sync_ref_time_).toSec()
                                      : nanValue();

            Eigen::Vector3d rel_pred = relativeToTarget(target_id);
            double rel_gt_x = nanValue();
            double rel_gt_y = nanValue();
            double rel_gt_z = nanValue();
            if (best_gt_valid_[uav_id_] && best_gt_valid_[target_id])
            {
                rel_gt_x = best_gt_positions_[target_id].x - best_gt_positions_[uav_id_].x;
                rel_gt_y = best_gt_positions_[target_id].y - best_gt_positions_[uav_id_].y;
                rel_gt_z = best_gt_positions_[target_id].z - best_gt_positions_[uav_id_].z;
            }

            const double rel_norm = rel_pred.norm();

            double uwb_meas = nanValue();
            double uwb_pred = rel_norm;
            double uwb_residual = nanValue();
            double cost_uwb = nanValue();
            const int uwb_idx = findUwbIndexForTarget(target_id);
            if (uwb_idx >= 0)
            {
                uwb_meas = best_uwb_.distances[uwb_idx];
                uwb_residual = uwb_meas - uwb_pred;
                const double uwb_stddev = currentUwbStddev();
                cost_uwb = sqr(uwb_residual / uwb_stddev);
            }

            double alpha_meas = nanValue();
            double alpha_pred = nanValue();
            double alpha_residual = nanValue();
            double theta_meas = nanValue();
            double theta_pred = nanValue();
            double theta_residual = nanValue();
            double cost_angle = nanValue();
            const int camera_idx = findCameraIndexForTarget(target_id);
            if (camera_idx >= 0 && rel_norm > 1e-6)
            {
                alpha_meas = best_camera_.alpha[camera_idx];
                theta_meas = best_camera_.theta[camera_idx];
                alpha_pred = std::atan2(rel_pred.y(), rel_pred.x());
                theta_pred = std::asin(std::max(-1.0, std::min(1.0, rel_pred.z() / rel_norm)));
                alpha_residual = normalizeAngle(alpha_meas - alpha_pred);
                theta_residual = theta_meas - theta_pred;
                cost_angle = sqr(alpha_residual / ANGLE_ALPHA_STDDEV) +
                             sqr(theta_residual / ANGLE_THETA_STDDEV);
            }

            double xy_residual_x = nanValue();
            double xy_residual_y = nanValue();
            double cost_xy = nanValue();
            if (uwb_idx >= 0 && camera_idx >= 0)
            {
                const double ca = std::cos(alpha_meas);
                const double sa = std::sin(alpha_meas);
                const double ct = std::cos(theta_meas);
                const double st = std::sin(theta_meas);
                const double proj = uwb_meas * ct;

                xy_residual_x = proj * ca - rel_pred.x();
                xy_residual_y = proj * sa - rel_pred.y();

                const double sigma_x2 = std::max(
                    sqr(ct * ca * currentUwbStddev()) +
                    sqr(-uwb_meas * ct * sa * ANGLE_ALPHA_STDDEV) +
                    sqr(-uwb_meas * st * ca * ANGLE_THETA_STDDEV),
                    0.05 * 0.05);
                const double sigma_y2 = std::max(
                    sqr(ct * sa * currentUwbStddev()) +
                    sqr(uwb_meas * ct * ca * ANGLE_ALPHA_STDDEV) +
                    sqr(-uwb_meas * st * sa * ANGLE_THETA_STDDEV),
                    0.05 * 0.05);

                cost_xy = xy_residual_x * xy_residual_x / sigma_x2 +
                          xy_residual_y * xy_residual_y / sigma_y2;
            }

            residual_csv_ << stamp << ','
                          << phase << ','
                          << uav_id_ << ','
                          << target_id << ','
                          << uwb_dt << ','
                          << camera_dt << ','
                          << com_dt << ','
                          << rel_gt_x << ','
                          << rel_gt_y << ','
                          << rel_gt_z << ','
                          << rel_pred.x() << ','
                          << rel_pred.y() << ','
                          << rel_pred.z() << ','
                          << uwb_meas << ','
                          << uwb_pred << ','
                          << uwb_residual << ','
                          << alpha_meas << ','
                          << alpha_pred << ','
                          << alpha_residual << ','
                          << theta_meas << ','
                          << theta_pred << ','
                          << theta_residual << ','
                          << xy_residual_x << ','
                          << xy_residual_y << ','
                          << cost_uwb << ','
                          << cost_angle << ','
                          << cost_xy << ','
                          << cost_ins << ','
                          << (camera_target_valid_[target_id] ? 1 : 0) << ','
                          << (angle_error_valid_[target_id] ? 1 : 0) << ','
                          << (dist_xy_error_valid_[target_id] ? 1 : 0) << '\n';
        }
        residual_csv_.flush();
    }

    bool isReadyForDGO()
    {
        // 1. 本机 INS 必须就绪
        if (!latest_ins_msg_)
            return false;

        // 地面阶段 UWB 距离约束缺乏足够方向信息，且各节点尚未形成稳定
        // 闭环。此时启动分布式优化会让整个编队沿欠约束方向共同漂移。
        // 首次起飞后锁存启动状态，降落阶段仍保持 DGO 连续运行。
        if (!dgo_started_)
        {
            if (use_gazebo_initial_offsets_ && !origin_locked_)
                return false;

            const double altitude =
                latest_ins_msg_->pose.pose.position.z;
            const bool mission_started = mission_stage_ >= 1;
            const bool altitude_ready =
                std::isfinite(altitude) &&
                altitude >= min_dgo_start_altitude_;
            if (require_airborne_initialization_ &&
                (!mission_started || !altitude_ready))
            {
                ROS_INFO_THROTTLE(
                    5.0,
                    "[iris_%d] DGO waiting for airborne initialization: "
                    "stage=%u altitude=%.3f required=%.3f origin_locked=%d",
                    uav_id_, static_cast<unsigned int>(mission_stage_),
                    altitude, min_dgo_start_altitude_,
                    origin_locked_ ? 1 : 0);
                return false;
            }

            P_opt_ = latest_ins_msg_->pose.pose.position;
            prev_P_opt_ = P_opt_;
            Pc[uav_id_] = P_opt_;
            has_prev_ins_pos_ = false;
            ins_delta_valid_ = false;
            ins_update_ = false;
            dgo_started_ = true;
            ROS_INFO("[iris_%d] DGO initialized from airborne INS: "
                     "stage=%u altitude=%.3f p=(%.3f %.3f %.3f)",
                     uav_id_, static_cast<unsigned int>(mission_stage_),
                     altitude, P_opt_.x, P_opt_.y, P_opt_.z);
        }

        // 2. 其他无人机通信要到齐且不能长期陈旧。不再要求每轮全员
        // has_new_com_，但旧数据不能无限参与参考时间和速度外推。
        const ros::Time now = ros::Time::now();
        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_)
                continue;
            if (!has_com_[i] || data_timestamps_[i].isZero())
                return false;
            const double age = std::fabs((now - data_timestamps_[i]).toSec());
            if (age > max_communication_age_)
            {
                ROS_WARN_THROTTLE(
                    10.0,
                    "[iris_%d] skip update: communication from iris_%d stale, "
                    "age=%.3fs > %.3fs",
                    uav_id_, i, age, max_communication_age_);
                return false;
            }
        }

        // 3. 计算同步参考时间 (其他机 data_timestamps_ 均值)
        if (!updateReferenceTime(sync_ref_time_))
            return false;

        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_)
                continue;
            const double skew =
                std::fabs((data_timestamps_[i] - sync_ref_time_).toSec());
            if (skew > max_communication_skew_)
            {
                ROS_WARN_THROTTLE(
                    10.0,
                    "[iris_%d] skip update: communication skew from iris_%d "
                    "%.3fs > %.3fs",
                    uav_id_, i, skew, max_communication_skew_);
                return false;
            }
        }

        // 4. GT 同样按参考时间对齐。正常模式用于诊断，Oracle 模式是硬门槛。
        const bool has_aligned_gt = selectNearestGt(sync_ref_time_);
        if (has_aligned_gt &&
            !isFresh(best_gt_stamp_, sync_ref_time_, max_gt_age_))
        {
            std::fill(best_gt_valid_.begin(), best_gt_valid_.end(), false);
        }
        if (oracle_mode_)
        {
            if (!origin_locked_ || !has_aligned_gt ||
                best_gt_valid_.empty() || !best_gt_valid_[uav_id_])
            {
                ROS_WARN_THROTTLE(
                    10.0,
                    "[iris_%d] Oracle waiting for locked origins and aligned GT",
                    uav_id_);
                return false;
            }
            for (int i = 0; i < uav_num_; ++i)
            {
                if (i != uav_id_ && !best_gt_valid_[i])
                    return false;
            }
        }

        // 5. 用通信位置 + 速度外推，或用对齐后的 Oracle 真值位移得到 Pc
        updatePredictedPeerPositions(sync_ref_time_);

        // 6. 按 sync_ref_time_ 重新选 UWB, 必须新鲜
        if (!selectNearestUwb(sync_ref_time_))
            return false;
        if (!isFresh(best_uwb_stamp_, sync_ref_time_))
        {
            ROS_WARN_THROTTLE(10.0,
                              "[iris_%d] skip update: UWB stale, age=%.3fs > %.3fs", uav_id_,
                              std::fabs((best_uwb_stamp_ - sync_ref_time_).toSec()),
                              max_sensor_age_);
            return false;
        }

        // 7. 相机可选: 选到且新鲜就用, 否则 UWB + INS
        // 重置每轮统计 (避免残留)
        camera_message_fresh_ = false;
        camera_raw_count_ = 0;
        camera_valid_target_count_ = 0;
        camera_angle_constraint_count_ = 0;
        camera_xy_constraint_count_ = 0;
        camera_used_in_cost_ = false;
        std::fill(camera_target_valid_.begin(), camera_target_valid_.end(), false);

        camera_message_fresh_ = has_camera_ &&
                                selectNearestCamera(sync_ref_time_) &&
                                isFresh(best_camera_stamp_, sync_ref_time_);
        if (camera_message_fresh_)
            camera_raw_count_ = static_cast<int>(best_camera_.count);
        if (has_camera_ && !camera_message_fresh_)
        {
            ROS_WARN_THROTTLE(10.0,
                              "[iris_%d] camera stale, use UWB+INS only, age=%.3fs > %.3fs", uav_id_,
                              std::fabs((best_camera_stamp_ - sync_ref_time_).toSec()),
                              max_sensor_age_);
        }

        // 8. 选本机 INS, 用于里程计约束 (Jo)
        if (!selectNearestIns(sync_ref_time_))
            return false;
        return true;
    }

    void initInitialOffsets()
    {
        initial_offsets_.assign(uav_num_, Eigen::Vector3d::Zero());
        for (int i = 0; i < uav_num_; ++i)
        {
            // Match outdoor2.launch clockwise arrangement:
            //   iris_0 (0,0), iris_1 (2,0), iris_2 (2,2), iris_3 (0,2)
            const double x = (i % 2) != (i / 2) ? initial_spacing_ : 0.0;
            const double y = (i / 2) * initial_spacing_;
            initial_offsets_[i] = Eigen::Vector3d(x, y, 0.0);
        }
    }

    bool updateReferenceTime(ros::Time &ref) const
    {
        double ref_sec = 0.0;
        int count = 0;
        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_)
                continue;
            const ros::Time &t = data_timestamps_[i];
            if (!t.isZero())
            {
                ref_sec += t.toSec();
                ++count;
            }
        }
        if (count == 0)
            return false;
        ref = ros::Time(ref_sec / count);
        return true;
    }

    bool isFresh(const ros::Time &stamp, const ros::Time &ref) const
    {
        return isFresh(stamp, ref, max_sensor_age_);
    }

    static bool isFresh(const ros::Time &stamp,
                        const ros::Time &ref,
                        double max_age)
    {
        if (stamp.isZero() || ref.isZero())
            return false;
        return std::fabs((stamp - ref).toSec()) <= max_age;
    }

    void updatePredictedPeerPositions(const ros::Time &ref)
    {
        // Oracle 模式只替换邻机的位置源，参考时间仍与正常模式一致，
        // 从而形成受控 A/B。Pc 的定义是“各机相对自身初始点的位移”，
        // 不能直接写入 Gazebo 世界坐标。
        if (oracle_mode_)
        {
            for (int i = 0; i < uav_num_; ++i)
            {
                if (i == uav_id_ || !best_gt_valid_[i])
                    continue;
                Pc[i].x = best_gt_positions_[i].x - origins_[i].x();
                Pc[i].y = best_gt_positions_[i].y - origins_[i].y();
                Pc[i].z = best_gt_positions_[i].z - origins_[i].z();
            }
            return;
        }

        // 正常模式: 通信位置 + INS 速度外推
        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_ || !has_com_[i] || data_timestamps_[i].isZero())
                continue;

            const double raw_dt = (ref - data_timestamps_[i]).toSec();
            const double dt = std::max(-max_extrapolation_dt_,
                                       std::min(max_extrapolation_dt_, raw_dt));
            Pc[i].x = com_positions_[i].x + com_velocities_[i].x * dt;
            Pc[i].y = com_positions_[i].y + com_velocities_[i].y * dt;
            Pc[i].z = com_positions_[i].z + com_velocities_[i].z * dt;
        }
    }

    bool selectNearestGt(const ros::Time &ref)
    {
        std::fill(best_gt_valid_.begin(), best_gt_valid_.end(), false);
        best_gt_stamp_ = ros::Time();
        if (ref.isZero() || gt_history_.empty())
            return false;

        double min_dt = std::numeric_limits<double>::max();
        const GtSample *best = nullptr;
        for (const auto &entry : gt_history_)
        {
            const double dt = std::fabs((entry.stamp - ref).toSec());
            if (dt < min_dt)
            {
                min_dt = dt;
                best = &entry;
            }
        }
        if (!best)
            return false;

        best_gt_positions_ = best->positions;
        best_gt_valid_ = best->valid;
        best_gt_stamp_ = best->stamp;
        return true;
    }

    // 按 ref 在 UWB 缓存中选时间戳最近的样本, 命中则更新 best_uwb_/stamp_, 返回 true
    bool selectNearestUwb(const ros::Time &ref)
    {
        if (ref.isZero() || uwb_history_.empty())
            return false;
        double min_dt = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto &entry : uwb_history_)
        {
            const double dt = std::fabs((entry.first - ref).toSec());
            if (dt < min_dt)
            {
                min_dt = dt;
                best_uwb_ = entry.second;
                best_uwb_stamp_ = entry.first;
                found = true;
            }
        }
        return found;
    }

    // 按 ref 在相机缓存中选时间戳最近的样本, 命中则更新 best_camera_/stamp_, 返回 true
    bool selectNearestCamera(const ros::Time &ref)
    {
        if (ref.isZero() || camera_history_.empty())
            return false;
        double min_dt = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto &entry : camera_history_)
        {
            const double dt = std::fabs((entry.first - ref).toSec());
            if (dt < min_dt)
            {
                min_dt = dt;
                best_camera_ = entry.second;
                best_camera_stamp_ = entry.first;
                found = true;
            }
        }
        return found;
    }

    // 按 ref 在 INS 缓存中选时间戳最近的样本, 命中则更新 best_ins_pos_, 返回 true
    bool selectNearestIns(const ros::Time &ref)
    {
        if (ref.isZero() || ins_history_.empty())
            return false;
        double min_dt = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto &entry : ins_history_)
        {
            const double dt = std::fabs((entry.first - ref).toSec());
            if (dt < min_dt)
            {
                min_dt = dt;
                best_ins_pos_ = entry.second;
                best_ins_stamp_ = entry.first;
                found = true;
            }
        }
        if (found)
        {
            has_ins_ = true;
            ins_update_ = true;
        }
        return found;
    }

    Eigen::Vector3d relativeToTarget(int target_id) const
    {
        Eigen::Vector3d offset =
            initial_offsets_[target_id] - initial_offsets_[uav_id_];
        Eigen::Vector3d delta(Pc[target_id].x - P_opt_.x,
                              Pc[target_id].y - P_opt_.y,
                              Pc[target_id].z - P_opt_.z);
        return offset + delta;
    }

    void publishDgoEstimate()
    {
        nav_msgs::Odometry msg;
        msg.header.stamp = sync_ref_time_.isZero() ? ros::Time::now() : sync_ref_time_;
        msg.header.frame_id = uavs_[uav_id_].substr(1) + "/initial_enu";
        msg.child_frame_id = uavs_[uav_id_].substr(1) + "/base_link";

        msg.pose.pose.position = P_opt_;
        if (latest_ins_msg_)
        {
            msg.pose.pose.orientation = latest_ins_msg_->pose.pose.orientation;
            msg.twist = latest_ins_msg_->twist;
            msg.pose.covariance = latest_ins_msg_->pose.covariance;
            msg.twist.covariance = latest_ins_msg_->twist.covariance;
        }
        else
        {
            msg.pose.pose.orientation.w = 1.0;
        }

        dgo_pub_.publish(msg);
    }
    /*DGO算法中误差函数和惩罚函数的计算*/
    bool prepareInsDelta()
    {
        if (!has_ins_)
            return false;

        if (!has_prev_ins_pos_)
        {
            prev_ins_pos_ = best_ins_pos_;
            has_prev_ins_pos_ = true;
            ins_delta_.x = 0.0;
            ins_delta_.y = 0.0;
            ins_delta_.z = 0.0;
            ins_update_ = false;
            return false;
        }

        if (!ins_update_)
            return false;

        ins_delta_.x = best_ins_pos_.x - prev_ins_pos_.x;
        ins_delta_.y = best_ins_pos_.y - prev_ins_pos_.y;
        ins_delta_.z = best_ins_pos_.z - prev_ins_pos_.z;
        return true;
    }

    void commitInsDelta()
    {
        if (!ins_delta_valid_)
            return;

        prev_ins_pos_ = best_ins_pos_;
        ins_update_ = false;
    }

    double cal_ins_penalty()
    {
        if(ins_delta_valid_)
        {
            double penalty = 0.0;
            geometry_msgs::Point delta;
            delta.x = ins_delta_.x - (P_opt_.x - prev_P_opt_.x);
            delta.y = ins_delta_.y - (P_opt_.y - prev_P_opt_.y);
            delta.z = ins_delta_.z - (P_opt_.z - prev_P_opt_.z);
            penalty +=
                (delta.x / ins_prior_stddev_xy_)*(delta.x / ins_prior_stddev_xy_) +
                (delta.y / ins_prior_stddev_xy_)*(delta.y / ins_prior_stddev_xy_);

            return penalty;
        }
        return 0.0;
    }

    void cal_dist_error()
    {
        std::fill(dist_error_.begin(), dist_error_.end(), 0.0);

        const size_t n = std::min(best_uwb_.target_ids.size(), best_uwb_.distances.size());
        for(size_t j = 0; j < n; j++)
        {
            int target_id = best_uwb_.target_ids[j];
            if(target_id < 0 || target_id >= uav_num_ || target_id == uav_id_)
            {
                continue;
            }

            Eigen::Vector3d rel = relativeToTarget(target_id);
            double dx = rel.x();
            double dy = rel.y();
            double dz = rel.z();

            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            double error = best_uwb_.distances[j] - dist;
            dist_error_[target_id] = error;
        }
    }

    double cal_dist_penalty()
    {
        double penalty = 0.0;
        const size_t n = std::min(best_uwb_.target_ids.size(), best_uwb_.distances.size());
        for(size_t j = 0; j < n; j++)
        {
            int target_id = best_uwb_.target_ids[j];
            if(target_id < 0 || target_id >= uav_num_ || target_id == uav_id_)
            {
                continue;
            }
            const double uwb_stddev = currentUwbStddev();
            penalty += (dist_error_[target_id] / uwb_stddev) *
                       (dist_error_[target_id] / uwb_stddev);
        }
        return penalty;
    }

    void cal_angle_error()
    {
        // 每次重算前清空 valid 标记, 避免残留值污染本轮 penalty
        std::fill(angle_error_valid_.begin(), angle_error_valid_.end(), false);

        if(camera_message_fresh_)
        {
            const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                       best_camera_.id.size(),
                                       best_camera_.alpha.size(),
                                       best_camera_.theta.size()});
            std::vector<bool> seen_target(uav_num_, false);
            for(size_t j = 0; j < n; ++j)
            {
                int target_id = -1;
                if (!isValidCameraTarget(j, target_id))
                    continue;
                if (seen_target[target_id])
                    continue;
                seen_target[target_id] = true;
                camera_target_valid_[target_id] = true;

                // 根据初始相对偏移 + 当前位移差计算相对角度
                Eigen::Vector3d rel = relativeToTarget(target_id);
                double dx = rel.x();
                double dy = rel.y();
                double dz = rel.z();

                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-6)
                    continue;

                double ratio = std::max(-1.0, std::min(1.0, dz / dist));
                double alpha_error = normalizeAngle(best_camera_.alpha[j] - std::atan2(dy,dx));
                double theta_error = best_camera_.theta[j] - std::asin(ratio);

                angle_err_[target_id].alpha = alpha_error;
                angle_err_[target_id].theta = theta_error;
                angle_error_valid_[target_id] = true;
            }
        }
    }

    double cal_angle_penalty()
    {
        if(!camera_message_fresh_)
            return 0.0;

        double penalty = 0.0;
        for (int target_id = 0; target_id < uav_num_; ++target_id)
        {
            if (!angle_error_valid_[target_id])
                continue;
            penalty += (angle_err_[target_id].alpha / ANGLE_ALPHA_STDDEV)*(angle_err_[target_id].alpha / ANGLE_ALPHA_STDDEV)
                        + (angle_err_[target_id].theta / ANGLE_THETA_STDDEV)*(angle_err_[target_id].theta / ANGLE_THETA_STDDEV);
        }
        return penalty;
    }

    void cal_dist_XY_error()
    {
        std::fill(dist_xy_error_valid_.begin(), dist_xy_error_valid_.end(), false);

        if(!has_uwb_ || !camera_message_fresh_)
            return;

        const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                   best_camera_.id.size(),
                                   best_camera_.alpha.size(),
                                   best_camera_.theta.size()});
        std::vector<bool> seen_target(uav_num_, false);
        for(size_t j = 0; j < n; ++j)
        {
            int target = -1;
            if (!isValidCameraTarget(j, target))
                continue;
            if (seen_target[target])
                continue;
            seen_target[target] = true;
            camera_target_valid_[target] = true;

            auto it = std::find(best_uwb_.target_ids.begin(), best_uwb_.target_ids.end(), target);
            if(it == best_uwb_.target_ids.end())
                continue;
            size_t idx = std::distance(best_uwb_.target_ids.begin(), it);
            if(idx >= best_uwb_.distances.size())
                continue;
            double d_uwb = best_uwb_.distances[idx];

            double alpha = best_camera_.alpha[j];
            double theta = best_camera_.theta[j];

            // 保护: 非有限值或距离太小 (<0.1m) 时跳过,
            //   proj = d * cos(theta) 在 distance 极小或方位角极大时投影不可靠
            if (!std::isfinite(d_uwb) || d_uwb < 0.1)
                continue;
            double ca = std::cos(alpha);
            double sa = std::sin(alpha);
            double ct = std::cos(theta);
            double st = std::sin(theta);

            double proj = d_uwb * ct;
            Eigen::Vector3d rel = relativeToTarget(target);
            dist_xy_error_[target].x = (proj * ca)
                                     - rel.x();
            dist_xy_error_[target].y = (proj * sa)
                                     - rel.y();

            double sigma_x2 =
                std::pow(ct * ca * currentUwbStddev(), 2) +
                std::pow(-d_uwb * ct * sa * ANGLE_ALPHA_STDDEV, 2) +
                std::pow(-d_uwb * st * ca * ANGLE_THETA_STDDEV, 2);

            double sigma_y2 =
                std::pow(ct * sa * currentUwbStddev(), 2) +
                std::pow(d_uwb * ct * ca * ANGLE_ALPHA_STDDEV, 2) +
                std::pow(-d_uwb * st * sa * ANGLE_THETA_STDDEV, 2);

            dist_xy_error_[target].sigma_x2 = std::max(sigma_x2, 0.05 * 0.05);
            dist_xy_error_[target].sigma_y2 = std::max(sigma_y2, 0.05 * 0.05);
            dist_xy_error_valid_[target] = true;
        }
    }

    double cal_dist_XY_penalty()
    {
        double penalty = 0.0;
        for(size_t j = 0; j < dist_xy_error_.size(); j++)
        {
            if(!dist_xy_error_valid_[j])
            {
                continue;
            }

            penalty += dist_xy_error_[j].x * dist_xy_error_[j].x / dist_xy_error_[j].sigma_x2
                     + dist_xy_error_[j].y * dist_xy_error_[j].y / dist_xy_error_[j].sigma_y2;
        }

        return penalty;
    }  

    CostBreakdown computeCostBreakdown()
    {
        // cal_cost() 会被数值梯度反复调用。每次都从干净状态开始，
        // 使 valid 数组只描述当前候选状态，不累积中间求值结果。
        std::fill(camera_target_valid_.begin(), camera_target_valid_.end(), false);
        std::fill(angle_error_valid_.begin(), angle_error_valid_.end(), false);
        std::fill(dist_xy_error_valid_.begin(), dist_xy_error_valid_.end(), false);

        CostBreakdown cb;
        cal_angle_error();
        cb.angle = cal_angle_penalty();
        cal_dist_error();
        cb.dist  = cal_dist_penalty();
        cal_dist_XY_error();
        cb.xy    = cal_dist_XY_penalty();
        cb.ins   = cal_ins_penalty();
        return cb;
    }

    double cal_cost(const Eigen::Vector2d& x)
    {
        geometry_msgs::Point old = P_opt_;
        P_opt_.x = x.x();
        P_opt_.y = x.y();
        // P_opt_.z 保持 RunDGO() 中由 INS 增量预测的固定值。

        //计算惩罚
        double cost = computeCostBreakdown().total();

        P_opt_ = old;
        return cost;
    }

    double cal_cost_grad(const Eigen::VectorXd& x,Eigen::VectorXd& grad)
    {
        if (x.size() != 2)
            throw std::runtime_error("DGO XY optimizer expected 2 variables");

        Eigen::Vector2d xv;
        xv << x[0],x[1];

        double fx = cal_cost(xv);
        grad.resize(2);
        const double eps = 1e-4;

        for(int k = 0;k<2;k++)
        {
            Eigen::Vector2d xp = xv;
            Eigen::Vector2d xm = xv;
            xp[k] += eps;
            xm[k] -= eps;
            double fp = cal_cost(xp);
            double fm = cal_cost(xm);
            grad[k] = (fp - fm) / (2.0*eps);
        }
        return fx;
    }

    static double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    //LBFGS Functor
    struct DGOFunctor
    {
        DGO * dgo;
        explicit DGOFunctor(DGO* ptr):dgo(ptr){}

        double operator()(const Eigen::VectorXd& x,Eigen::VectorXd& grad)
        {
            return dgo->cal_cost_grad(x,grad);
        }
    };
};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"DGO");
    DGO d;
    ros::spin();
    return 0;
}
