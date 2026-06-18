#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

class EkfDgoTest
{
public:
    EkfDgoTest()
        : nh_(), pnh_("~")
    {
        pnh_.param("self_name", self_name_, inferSelfName());
        pnh_.param("max_align_dt", max_align_dt_, 0.08);
        pnh_.param("max_dgo_align_dt", max_dgo_align_dt_, 0.12);
        pnh_.param("max_model_align_dt", max_model_align_dt_, 0.03);
        pnh_.param("initial_spacing", initial_spacing_, 2.0);
        pnh_.param("csv_dir", csv_dir_, std::string("/home/scott/swarm_localization/src/test/logs"));
        reference_name_ = "iris_0";
        ensureOutputDir();

        self_dgo_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/" + self_name_ + "/dgo_estimate", 10,
            &EkfDgoTest::selfDgoCallback, this);

        reference_dgo_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/" + reference_name_ + "/dgo_estimate", 10,
            &EkfDgoTest::referenceDgoCallback, this);

        model_states_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", 10,
            &EkfDgoTest::modelStatesCallback, this);

        ROS_INFO("[EKF DGO TEST] self=%s reference=%s max_dgo_align_dt=%.3f max_model_align_dt=%.3f subscribed to /%s/dgo_estimate, /%s/dgo_estimate, /gazebo/model_states",
                 self_name_.c_str(), reference_name_.c_str(),
                 max_dgo_align_dt_, max_model_align_dt_,
                 self_name_.c_str(), reference_name_.c_str());
    }

    ~EkfDgoTest()
    {
        shutdownAndReport();
    }

    void shutdownAndReport()
    {
        if (reported_)
            return;
        reported_ = true;

        writeErrorCSV();

        if (sample_count_ == 0)
        {
            fprintf(stdout,
                    "\n[EKF DGO TEST] %s relative to %s: no valid samples; RMSE unavailable\n",
                    self_name_.c_str(), reference_name_.c_str());
            fprintf(stdout,
                    "self_dgo_callbacks=%lu, reject_reference=%lu, reject_model=%lu, "
                    "last_ref_dt=%.6f s, last_model_dt=%.6f s\n",
                    static_cast<unsigned long>(self_dgo_callbacks_),
                    static_cast<unsigned long>(reject_reference_count_),
                    static_cast<unsigned long>(reject_model_count_),
                    last_reference_dt_, last_model_dt_);
            fflush(stdout);
            return;
        }

        const double rmse = std::sqrt(error_ / static_cast<double>(sample_count_));
        fprintf(stdout, "\n============================================\n");
        fprintf(stdout, "[EKF DGO TEST] %s relative to %s\n",
                self_name_.c_str(), reference_name_.c_str());
        fprintf(stdout, "samples: %lu\n", static_cast<unsigned long>(sample_count_));
        fprintf(stdout, "RMSE: %.6f m\n", rmse);
        fprintf(stdout,
                "self_dgo_callbacks=%lu, reject_reference=%lu, reject_model=%lu\n",
                static_cast<unsigned long>(self_dgo_callbacks_),
                static_cast<unsigned long>(reject_reference_count_),
                static_cast<unsigned long>(reject_model_count_));
        fprintf(stdout, "============================================\n");
        fflush(stdout);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber self_dgo_sub_;
    ros::Subscriber reference_dgo_sub_;
    ros::Subscriber model_states_sub_;
    std::string self_name_;
    std::string reference_name_;
    std::string csv_dir_;
    double max_align_dt_ = 0.08;
    double max_dgo_align_dt_ = 0.12;
    double max_model_align_dt_ = 0.03;
    double initial_spacing_ = 2.0;

    struct DgoSample
    {
        ros::Time stamp;
        nav_msgs::Odometry data;
    };

    struct ModelPoseSample
    {
        ros::Time stamp;
        geometry_msgs::Pose self_pose;
        geometry_msgs::Pose reference_pose;
    };

    struct ErrorSample
    {
        double timestamp = 0.0;
        double x_gt = 0.0;
        double y_gt = 0.0;
        double z_gt = 0.0;
        double x_est = 0.0;
        double y_est = 0.0;
        double z_est = 0.0;
        double err_x = 0.0;
        double err_y = 0.0;
        double err_z = 0.0;
        double error_norm = 0.0;
        double cumulative_rmse = 0.0;
    };

    static constexpr size_t kMaxCacheSize = 200;
    nav_msgs::Odometry self_dgo_data_;
    nav_msgs::Odometry reference_dgo_data_;
    ModelPoseSample model_pose_data_;
    std::deque<DgoSample> reference_dgo_cache_;
    std::deque<ModelPoseSample> model_pose_cache_;
    bool has_self_dgo_ = false;
    bool has_reference_dgo_ = false;
    bool has_model_pose_ = false;
    double error_ = 0.0;
    size_t sample_count_ = 0;
    size_t self_dgo_callbacks_ = 0;
    size_t reject_reference_count_ = 0;
    size_t reject_model_count_ = 0;
    double last_reference_dt_ = -1.0;
    double last_model_dt_ = -1.0;
    std::vector<ErrorSample> error_samples_;
    bool reported_ = false;

    std::string inferSelfName() const
    {
        std::string ns = ros::this_node::getNamespace();
        if (!ns.empty() && ns[0] == '/')
            ns.erase(0, 1);
        const size_t slash = ns.find('/');
        if (slash != std::string::npos)
            ns = ns.substr(0, slash);
        return ns.empty() ? std::string("iris_1") : ns;
    }

    void selfDgoCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        self_dgo_data_ = *msg;
        has_self_dgo_ = true;
        ++self_dgo_callbacks_;

        const ros::Time ref_stamp = msg->header.stamp;
        if (!selectNearestDgo(reference_dgo_cache_, ref_stamp, max_dgo_align_dt_,
                              reference_dgo_data_, last_reference_dt_))
        {
            ++reject_reference_count_;
            has_reference_dgo_ = false;
            return;
        }

        if (!selectNearestModelPose(ref_stamp, model_pose_data_, last_model_dt_))
        {
            ++reject_model_count_;
            has_model_pose_ = false;
            return;
        }

        has_reference_dgo_ = true;
        has_model_pose_ = true;

        const double x_gt = model_pose_data_.self_pose.position.x -
                            model_pose_data_.reference_pose.position.x;
        const double y_gt = model_pose_data_.self_pose.position.y -
                            model_pose_data_.reference_pose.position.y;
        const double z_gt = model_pose_data_.self_pose.position.z -
                            model_pose_data_.reference_pose.position.z;

        const geometry_msgs::Point self_offset = initialOffset(self_name_);
        const geometry_msgs::Point reference_offset = initialOffset(reference_name_);
        const double x_est = (self_offset.x + self_dgo_data_.pose.pose.position.x) -
                             (reference_offset.x + reference_dgo_data_.pose.pose.position.x);
        const double y_est = (self_offset.y + self_dgo_data_.pose.pose.position.y) -
                             (reference_offset.y + reference_dgo_data_.pose.pose.position.y);
        const double z_est = (self_offset.z + self_dgo_data_.pose.pose.position.z) -
                             (reference_offset.z + reference_dgo_data_.pose.pose.position.z);

        const double err_x = x_gt - x_est;
        const double err_y = y_gt - y_est;
        const double err_z = z_gt - z_est;
        const double err_sq = err_x * err_x + err_y * err_y + err_z * err_z;

        error_ += err_sq;
        ++sample_count_;

        ErrorSample sample;
        sample.timestamp = ref_stamp.toSec();
        sample.x_gt = x_gt;
        sample.y_gt = y_gt;
        sample.z_gt = z_gt;
        sample.x_est = x_est;
        sample.y_est = y_est;
        sample.z_est = z_est;
        sample.err_x = err_x;
        sample.err_y = err_y;
        sample.err_z = err_z;
        sample.error_norm = std::sqrt(err_sq);
        sample.cumulative_rmse = std::sqrt(error_ / static_cast<double>(sample_count_));
        error_samples_.push_back(sample);
    }

    void referenceDgoCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        reference_dgo_cache_.push_back({msg->header.stamp, *msg});
        trimCache(reference_dgo_cache_);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        const int self_idx = findModelIndex(msg, self_name_);
        const int reference_idx = findModelIndex(msg, reference_name_);
        if (self_idx < 0 || reference_idx < 0)
            return;

        ModelPoseSample sample;
        sample.stamp = ros::Time::now();
        sample.self_pose = msg->pose[self_idx];
        sample.reference_pose = msg->pose[reference_idx];

        model_pose_cache_.push_back(sample);
        trimCache(model_pose_cache_);
    }

    template <typename T>
    void trimCache(std::deque<T>& cache)
    {
        while (cache.size() > kMaxCacheSize)
            cache.pop_front();
    }

    int findModelIndex(const gazebo_msgs::ModelStates::ConstPtr& msg,
                       const std::string& name) const
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == name)
                return static_cast<int>(i);
        }
        return -1;
    }

    int parseUavIndex(const std::string& name) const
    {
        const std::string prefix = "iris_";
        const size_t pos = name.find(prefix);
        if (pos == std::string::npos)
            return 0;

        try
        {
            return std::stoi(name.substr(pos + prefix.size()));
        }
        catch (...)
        {
            return 0;
        }
    }

    geometry_msgs::Point initialOffset(const std::string& name) const
    {
        const int idx = parseUavIndex(name);
        geometry_msgs::Point p;
        p.x = ((idx % 2) != (idx / 2)) ? initial_spacing_ : 0.0;
        p.y = (idx / 2) * initial_spacing_;
        p.z = 0.0;
        return p;
    }

    bool selectNearestDgo(const std::deque<DgoSample>& cache,
                          const ros::Time& ref_stamp,
                          double max_dt,
                          nav_msgs::Odometry& out,
                          double& nearest_dt) const
    {
        if (ref_stamp.isZero() || cache.empty())
        {
            nearest_dt = -1.0;
            return false;
        }

        double best_dt = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < cache.size(); ++i)
        {
            const double dt = std::fabs((cache[i].stamp - ref_stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                best_idx = i;
            }
        }

        nearest_dt = best_dt;
        if (best_dt > max_dt)
            return false;

        out = cache[best_idx].data;
        return true;
    }

    bool selectNearestModelPose(const ros::Time& ref_stamp,
                                ModelPoseSample& out,
                                double& nearest_dt) const
    {
        if (ref_stamp.isZero() || model_pose_cache_.empty())
        {
            nearest_dt = -1.0;
            return false;
        }

        double best_dt = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < model_pose_cache_.size(); ++i)
        {
            const double dt = std::fabs((model_pose_cache_[i].stamp - ref_stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                best_idx = i;
            }
        }

        nearest_dt = best_dt;
        if (best_dt > max_model_align_dt_)
            return false;

        out = model_pose_cache_[best_idx];
        return true;
    }

    void ensureOutputDir() const
    {
        if (csv_dir_.empty() || csv_dir_ == ".")
            return;

        struct stat st;
        if (stat(csv_dir_.c_str(), &st) == 0)
        {
            if (S_ISDIR(st.st_mode))
                return;
            ROS_WARN("[EKF DGO TEST] csv_dir exists but is not a directory: %s", csv_dir_.c_str());
            return;
        }

        if (mkdir(csv_dir_.c_str(), 0755) != 0 && errno != EEXIST)
        {
            ROS_WARN("[EKF DGO TEST] failed to create csv_dir: %s", csv_dir_.c_str());
        }
    }

    void writeErrorCSV() const
    {
        const std::string path = csv_dir_ + "/" + self_name_ + "_relative_to_" +
                                 reference_name_ + "_dgo_error.csv";
        std::ofstream f(path);
        if (!f.is_open())
        {
            ROS_WARN("[EKF DGO TEST] cannot write error CSV: %s", path.c_str());
            return;
        }

        f << std::fixed << std::setprecision(9);
        f << "timestamp,x_gt,y_gt,z_gt,x_est,y_est,z_est,"
          << "err_x,err_y,err_z,error_norm_m,cumulative_rmse_m\n";
        for (const auto& s : error_samples_)
        {
            f << s.timestamp << ","
              << s.x_gt << "," << s.y_gt << "," << s.z_gt << ","
              << s.x_est << "," << s.y_est << "," << s.z_est << ","
              << s.err_x << "," << s.err_y << "," << s.err_z << ","
              << s.error_norm << "," << s.cumulative_rmse << "\n";
        }
        f.close();

        ROS_INFO("[EKF DGO TEST] saved error CSV: %s (%zu samples)",
                 path.c_str(), error_samples_.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_dgo_test", ros::init_options::NoSigintHandler);
    static volatile bool shutdown_requested = false;
    std::signal(SIGINT, [](int) { shutdown_requested = true; });

    EkfDgoTest node;
    while (ros::ok() && !shutdown_requested)
        ros::spinOnce();

    fprintf(stdout, "\n=== Ctrl-C received, computing EKF+DGO relative RMSE ===\n");
    fflush(stdout);
    node.shutdownAndReport();
    ros::shutdown();
    return 0;
}
