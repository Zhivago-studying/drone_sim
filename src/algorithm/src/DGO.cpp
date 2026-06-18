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
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <data_process/UwbProcessed.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <LBFGS.h>

#define UWB_STDDEV 0.08
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
        gt_positions_.resize(uav_num_);
        has_gt_.resize(uav_num_, false);
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

        dgo_pub_ = nh_.advertise<nav_msgs::Odometry>("dgo_estimate", 10);
        initResidualCsv();
        initCommDebugCsv();

        ROS_INFO("[DGO] ns=%s uav_id=%d uav_num=%d initial_spacing=%.2f max_sensor_age=%.2f",
                 ros::this_node::getNamespace().c_str(), uav_id_, uav_num_,
                 initial_spacing_, max_sensor_age_);
    }

    ~DGO()
    {
        if (residual_csv_.is_open())
            residual_csv_.close();
        if (comm_debug_csv_.is_open())
            comm_debug_csv_.close();
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

        // 缓存 INS 位置, 用于后续按时间戳挑选
        ins_history_.push_back({msg->header.stamp, P_i});
        if (ins_history_.size() > 20)
            ins_history_.pop_front();

        // 以其他无人机的 data_timestamps_ 平均值为参考时间
        double ref_sec = 0.0;
        int count = 0;
        for (int i = 0; i < uav_num_; i++)
        {
            if (i == uav_id_)
                continue;
            const auto &t = data_timestamps_[i];
            if (!t.isZero())
            {
                ref_sec += t.toSec();
                ++count;
            }
        }
        if (count > 0)
        {
            ros::Time ref(ref_sec / count);
            double min_dt = std::numeric_limits<double>::max();
            for (const auto &entry : ins_history_)
            {
                double dt = std::fabs((entry.first - ref).toSec());
                if (dt < min_dt)
                    {
                        min_dt = dt;
                        best_ins_pos_ = entry.second;
                        has_ins_ = true;
                        ins_update_ = true;
                    }
                }
            }

        //标记本地无人机INS数据是否收到
        has_com_[uav_id_] = true;
        //判断本地INS是否更新
        if(msg->header.seq != last_seq_[uav_id_])
        {
            last_seq_[uav_id_] = msg->header.seq;
            has_new_com_[uav_id_] = true;
        }
        else
        {
            has_new_com_[uav_id_] = false;
        }
    }

    void uwbCallback(const data_process::UwbProcessed::ConstPtr &msg)
    {
        // 缓存本条 UWB 数据用于后续挑选
        uwb_history_.push_back({msg->header.stamp, *msg});
        if (uwb_history_.size() > 20)
            uwb_history_.pop_front();

        // 以所有 data_timestamps_ 的平均值作为参考时间
        double ref_sec = 0.0;
        int count = 0;
        for (int i = 0; i < uav_num_; i++)
        {
            if (i == uav_id_)
                continue;

            const auto &t = data_timestamps_[i];
            if (!t.isZero())
            {
                ref_sec += t.toSec();
                ++count;
            }
        }
        if (count == 0)
            return;
        ros::Time ref(ref_sec / count);

        // 在缓冲区中挑选时间戳最接近参考时间的数据
        double min_dt = std::numeric_limits<double>::max();
        for (const auto &entry : uwb_history_)
        {
            double dt = std::fabs((entry.first - ref).toSec());
            if (dt < min_dt)
            {
                min_dt = dt;
                best_uwb_ = entry.second;
                best_uwb_stamp_ = entry.first;
                has_uwb_ = true;
            }
        }
    }

    void cameraCallback(const data_process::CameraAngleMatch::ConstPtr &msg)
    {
        // 缓存本条相机 ID 匹配角度数据用于后续挑选
        camera_history_.push_back({msg->header.stamp, *msg});
        if (camera_history_.size() > 20)
            camera_history_.pop_front();

        // 以所有 data_timestamps_ 的平均值作为参考时间
        double ref_sec = 0.0;
        int count = 0;
        for (int i = 0; i < uav_num_; i++)
        {
            if (i == uav_id_)
                continue;

            const auto &t = data_timestamps_[i];
            if (!t.isZero())
            {
                ref_sec += t.toSec();
                ++count;
            }
        }
        if (count == 0)
            return;
        ros::Time ref(ref_sec / count);

        // 在缓冲区中挑选时间戳最接近参考时间的数据
        double min_dt = std::numeric_limits<double>::max();
        for (const auto &entry : camera_history_)
        {
            double dt = std::fabs((entry.first - ref).toSec());
            if (dt < min_dt)
            {
                min_dt = dt;
                best_camera_ = entry.second;
                best_camera_stamp_ = entry.first;
                has_camera_ = true;
            }
        }
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
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
        }
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

        Eigen::VectorXd x(3);
        x << P_opt_.x, P_opt_.y, P_opt_.z;

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
            writeCommDebugCsv();

            int niter = solver.minimize(functor, x, final_cost);

            P_opt_.x = x[0];
            P_opt_.y = x[1];
            P_opt_.z = x[2];
            commitInsDelta();

            CostBreakdown after_cb = computeCostBreakdown();
            writeResidualDebugCsv("post", after_cb.ins);
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
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::vector<ros::Subscriber> com_subs_;
    ros::Subscriber ins_sub_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber model_sub_;
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

    // residual debug CSV
    std::string csv_dir_;
    std::ofstream residual_csv_;

    // communication vs GT debug CSV
    std::ofstream comm_debug_csv_;

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
    ros::Time sync_ref_time_;
    bool use_camera_in_cost_ = false;

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
        struct stat st;
        if (stat(dir.c_str(), &st) == 0)
            return S_ISDIR(st.st_mode);

        if (mkdir(dir.c_str(), 0755) == 0)
            return true;

        return errno == EEXIST;
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
                      << "cost_uwb,cost_angle,cost_xy,cost_ins\n";
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

            if (has_gt_[uav_id_] && has_gt_[target_id])
            {
                const Eigen::Vector3d target_global(
                    initial_offsets_[target_id].x() + Pc[target_id].x,
                    initial_offsets_[target_id].y() + Pc[target_id].y,
                    initial_offsets_[target_id].z() + Pc[target_id].z);
                com_error_x = target_global.x() - gt_positions_[target_id].x;
                com_error_y = target_global.y() - gt_positions_[target_id].y;
                com_error_z = target_global.z() - gt_positions_[target_id].z;
                com_error_norm = std::sqrt(com_error_x * com_error_x +
                                          com_error_y * com_error_y +
                                          com_error_z * com_error_z);
            }

            const Eigen::Vector3d target_global(
                initial_offsets_[target_id].x() + Pc[target_id].x,
                initial_offsets_[target_id].y() + Pc[target_id].y,
                initial_offsets_[target_id].z() + Pc[target_id].z);
            comm_debug_csv_ << stamp << ','
                            << uav_id_ << ','
                            << target_id << ','
                            << target_global.x() << ','
                            << target_global.y() << ','
                            << target_global.z() << ','
                            << (has_gt_[target_id] ? gt_positions_[target_id].x : nanValue()) << ','
                            << (has_gt_[target_id] ? gt_positions_[target_id].y : nanValue()) << ','
                            << (has_gt_[target_id] ? gt_positions_[target_id].z : nanValue()) << ','
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
        if (!use_camera_in_cost_)
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
        const double camera_dt = (use_camera_in_cost_ && !best_camera_stamp_.isZero() && !sync_ref_time_.isZero())
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
            if (has_gt_[uav_id_] && has_gt_[target_id])
            {
                rel_gt_x = gt_positions_[target_id].x - gt_positions_[uav_id_].x;
                rel_gt_y = gt_positions_[target_id].y - gt_positions_[uav_id_].y;
                rel_gt_z = gt_positions_[target_id].z - gt_positions_[uav_id_].z;
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
                cost_uwb = sqr(uwb_residual / UWB_STDDEV);
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
                    sqr(ct * ca * UWB_STDDEV) +
                    sqr(-uwb_meas * ct * sa * ANGLE_ALPHA_STDDEV) +
                    sqr(-uwb_meas * st * ca * ANGLE_THETA_STDDEV),
                    0.05 * 0.05);
                const double sigma_y2 = std::max(
                    sqr(ct * sa * UWB_STDDEV) +
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
                          << cost_ins << '\n';
        }
        residual_csv_.flush();
    }

    bool isReadyForDGO()
    {
        if(!has_uwb_)
        {
            return false;
        }

        for(int i = 0;i < uav_num_;i++)
        {
            if(!has_com_[i] || !has_new_com_[i])
            {
                return false;
            }
        }
        if (!updateReferenceTime(sync_ref_time_))
        {
            return false;
        }
        updatePredictedPeerPositions(sync_ref_time_);
        if (!isFresh(best_uwb_stamp_, sync_ref_time_))
        {
            ROS_WARN_THROTTLE(10.0,
                              "[iris_%d] skip update: UWB stale, age=%.3fs > %.3fs", uav_id_,
                              std::fabs((best_uwb_stamp_ - sync_ref_time_).toSec()),
                              max_sensor_age_);
            return false;
        }

        use_camera_in_cost_ = has_camera_ && isFresh(best_camera_stamp_, sync_ref_time_);
        if (has_camera_ && !use_camera_in_cost_)
        {
            ROS_WARN_THROTTLE(10.0,
                              "[iris_%d] camera stale, use UWB+INS only, age=%.3fs > %.3fs", uav_id_,
                              std::fabs((best_camera_stamp_ - sync_ref_time_).toSec()),
                              max_sensor_age_);
        }
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
        if (stamp.isZero() || ref.isZero())
            return false;
        return std::fabs((stamp - ref).toSec()) <= max_sensor_age_;
    }

    void updatePredictedPeerPositions(const ros::Time &ref)
    {
        for (int i = 0; i < uav_num_; ++i)
        {
            if (i == uav_id_ || !has_com_[i] || data_timestamps_[i].isZero())
                continue;

            const double dt = (ref - data_timestamps_[i]).toSec();
            Pc[i].x = com_positions_[i].x + com_velocities_[i].x * dt;
            Pc[i].y = com_positions_[i].y + com_velocities_[i].y * dt;
            Pc[i].z = com_positions_[i].z + com_velocities_[i].z * dt;
        }
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
                (delta.x / EKF_X_STDDEV)*(delta.x / EKF_X_STDDEV) +
                (delta.y / EKF_Y_STDDEV)*(delta.y / EKF_Y_STDDEV) +
                (delta.z / EKF_Z_STDDEV)*(delta.z / EKF_Z_STDDEV);

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
            penalty += (dist_error_[target_id] / UWB_STDDEV) * (dist_error_[target_id] / UWB_STDDEV);
        }
        return penalty;
    }

    void cal_angle_error()
    {
        if(use_camera_in_cost_)
        {
            //收到相机数据，开始计算角度误差
            const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                       best_camera_.id.size(),
                                       best_camera_.alpha.size(),
                                       best_camera_.theta.size()});
            for(int j = 0;j < n;j++)
            {
                int target_id = best_camera_.id[j];
                if(target_id < 0 || target_id >= uav_num_ || target_id == uav_id_)
                    continue;

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
            }
        }
    }

    double cal_angle_penalty()
    {
        if(!use_camera_in_cost_)
            return 0.0;

        double penalty = 0.0;
        const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                   best_camera_.id.size(),
                                   best_camera_.alpha.size(),
                                   best_camera_.theta.size()});
        for(size_t j = 0; j < n; j++)
        {
            int target_id = best_camera_.id[j];
            if(target_id < 0 || target_id >= uav_num_ || target_id == uav_id_)
            {
                continue;
            }
            penalty += (angle_err_[target_id].alpha / ANGLE_ALPHA_STDDEV)*(angle_err_[target_id].alpha / ANGLE_ALPHA_STDDEV)
                        + (angle_err_[target_id].theta / ANGLE_THETA_STDDEV)*(angle_err_[target_id].theta / ANGLE_THETA_STDDEV);
        }
        return penalty;
    }

    void cal_dist_XY_error()
    {
        std::fill(dist_xy_error_valid_.begin(), dist_xy_error_valid_.end(), false);

        if(!has_uwb_ || !use_camera_in_cost_)
            return;

        const size_t n = std::min({static_cast<size_t>(best_camera_.count),
                                   best_camera_.id.size(),
                                   best_camera_.alpha.size(),
                                   best_camera_.theta.size()});
        for(size_t j = 0; j < n; j++)
        {
            int target = best_camera_.id[j];
            if(target < 0 || target >= uav_num_ || target == uav_id_)
                continue;

            auto it = std::find(best_uwb_.target_ids.begin(), best_uwb_.target_ids.end(), target);
            if(it == best_uwb_.target_ids.end())
                continue;
            size_t idx = std::distance(best_uwb_.target_ids.begin(), it);
            if(idx >= best_uwb_.distances.size())
                continue;
            double d_uwb = best_uwb_.distances[idx];

            double alpha = best_camera_.alpha[j];
            double theta = best_camera_.theta[j];
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
                std::pow(ct * ca * UWB_STDDEV, 2) +
                std::pow(-d_uwb * ct * sa * ANGLE_ALPHA_STDDEV, 2) +
                std::pow(-d_uwb * st * ca * ANGLE_THETA_STDDEV, 2);

            double sigma_y2 =
                std::pow(ct * sa * UWB_STDDEV, 2) +
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

    double cal_cost(const Eigen::Vector3d& x)
    {
        geometry_msgs::Point old = P_opt_;
        P_opt_.x = x.x();
        P_opt_.y = x.y();
        P_opt_.z = x.z();

        //计算惩罚
        double cost = computeCostBreakdown().total();

        P_opt_ = old;
        return cost;
    }

    double cal_cost_grad(const Eigen::VectorXd& x,Eigen::VectorXd& grad)
    {
        Eigen::Vector3d xv;
        xv << x[0],x[1],x[2];

        double fx = cal_cost(xv);
        grad.resize(3);
        const double eps = 1e-4;

        for(int k = 0;k<3;k++)
        {
            Eigen::Vector3d xp = xv;
            Eigen::Vector3d xm = xv;
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
