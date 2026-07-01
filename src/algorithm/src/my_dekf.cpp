#include <ros/ros.h>

#include <data_process/CameraAngleMatch.h>
#include <data_process/UwbProcessed.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <Eigen/Dense>
#include <deque>
#include <string>
#include <vector>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class DEKF
{
public:
    DEKF() : nh_(), pnh_("~")
    {
        pnh_.param("uav_id", uav_id_, 0);
        pnh_.param("uav_num", uav_num_, 4);
        pnh_.param("init_pos_std", init_pos_std_, 0.75);
        pnh_.param("init_vel_std", init_vel_std_, 0.50);
        pnh_.param("sigma_ax", sigma_ax_, 0.80);
        pnh_.param("sigma_ay", sigma_ay_, 0.80);
        pnh_.param("sigma_az", sigma_az_, 0.80);

        Q_.setZero();
        Q_(0, 0) = sigma_ax_ * sigma_ax_;
        Q_(1, 1) = sigma_ay_ * sigma_ay_;
        Q_(2, 2) = sigma_az_ * sigma_az_;

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

        for (int i = 0; i < uav_num_; ++i)
        {
            std::string topic = "/iris_" + std::to_string(i) + "/dgo_estimate";
            dgo_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    topic, 10,
                    boost::bind(&DEKF::dgoCallback, this, _1, i)));
        }

        uwb_sub_ = nh_.subscribe<data_process::UwbProcessed>(
            "uwb_processed", 10, &DEKF::uwbCallback, this);

        camera_sub_ = nh_.subscribe<data_process::CameraAngleMatch>(
            "camera_angle_match", 10, &DEKF::cameraCallback, this);

        pnh_.param("rate", rate_, 30.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_),
                                 &DEKF::dekfCallback, this);
    }

private:
    void dgoCallback(const nav_msgs::Odometry::ConstPtr &msg, int id)
    {
        DGOSample sample;
        sample.msg = *msg;
        sample.stamp = msg->header.stamp;
        //存储进历史缓存
        dgo_cache_[id].push_back(sample);
        //如果超过最大存储空间则pop
        if(dgo_cache_[id].size() > 100)
        {
            dgo_cache_[id].pop_front();
        }
    }

    void uwbCallback(const data_process::UwbProcessed::ConstPtr &msg)
    {
        (void)msg;
    }

    void cameraCallback(const data_process::CameraAngleMatch::ConstPtr &msg)
    {
        (void)msg;
    }

    void dekfCallback(const ros::TimerEvent &event)
    {
        (void)event;
        const ros::Time now = ros::Time::now();
        for (int target = 0; target < uav_num_; ++target)
        {
            if (target == uav_id_)
                continue;
            Filter &f = filters_[target];
            //预测
            predict(f,now);
            //发布状态变量和协方差矩阵
            publishFilter(f);
        }
    }

    //构建观测矩阵
    void buildDgoObservationMatrix(int target_id)
    {
        
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer timer_;
    double rate_ = 30.0;
    double init_pos_std_ = 0.75;
    double init_vel_std_ = 0.50;

    int uav_id_ = 0;
    int uav_num_ = 4;
    ros::Time last_predict_stamp_;

    std::vector<ros::Subscriber> dgo_subs_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;

    // 过程噪声
    double sigma_ax_ = 0.80;
    double sigma_ay_ = 0.80;
    double sigma_az_ = 0.80;
    Eigen::Matrix3d Q_;

    struct History_Record
    {
        ros::Time stamp;
        Vector6d X_pred;    //X_k+1|k
        Matrix6d P_pred;    //P_k+1|k
        Matrix6d A;
        Matrix6d I_KH;
    };

    // 滤波器: 本机与 target_id 之间的相对位姿/速度估计
    struct Filter
    {
        int target_id = -1;
        bool initialized = false;
        ros::Time stamp;
        Eigen::Matrix<double, 6, 1> X = Eigen::Matrix<double, 6, 1>::Zero();   // [px, py, pz, vx, vy, vz]
        Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Identity(); // 协方差矩阵

        std::deque<History_Record> cache;
        ros::Publisher pub;
    };

    //DGO历史观测
    struct DGOSample
    {
        ros::Time stamp;
        nav_msgs::Odometry msg;
    };

    std::vector<Filter> filters_;
    std::vector<std::deque<DGOSample>> dgo_cache_;

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

    //预测函数
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

        //构造A
        History_Record rec;
        rec.stamp = stamp;
        rec.A.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        rec.A.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        rec.A.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
        rec.A.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

        //构造G
        Eigen::Matrix<double,6,3> G;
        G.block<3,3>(0,0) = 0.5 * Eigen::Matrix3d::Identity() * dt * dt;
        G.block<3,3>(3,0) = Eigen::Matrix3d::Identity() * dt;

        //计算: X_k+1|k = A_k * X_k
        rec.X_pred = rec.A * f.X;
        //计算：P_k+1|k = A_k * P_k * A_K.T + G_K * Q_k * G_k.T
        rec.P_pred = rec.A * f.P * rec.A.transpose() + G * Q_ * G.transpose();

        //更新滤波器状态
        f.X = rec.X_pred;
        f.P = rec.P_pred;
        f.stamp = stamp;

        //缓存历史
        f.cache.push_back(rec);

        //判断缓存是否超过长度
        CleanCacheOverflow(f);
    }

    // 清理过期的预测缓存记录, 控制内存增长
    void CleanCacheOverflow(Filter &f)
    {
        constexpr size_t kMaxCacheSize = 500;
        while (f.cache.size() > kMaxCacheSize)
        {
            f.cache.pop_front();
        }
    }

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_dekf");
    DEKF node;
    ros::spin();
    return 0;
}
