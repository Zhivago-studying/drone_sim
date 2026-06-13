#include <ros/ros.h>
#include <sensors/ComMsg.h>
#include <data_process/CameraAngleMatch.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <data_process/UwbProcessed.h>

#define DELTA_TIME 0.1
#define UWB_STDDEV 0.08
#define ANGLE_ALPHA_STDDEV 0.02725
#define ANGLE_THETA_STDDEV 0.0379
class DGO
{
public:
    DGO()
    {
        nh_.param<int>("uav_id",uav_id_,0);
        nh_.param<int>("uav_num",uav_num_,4);

        Pc.resize(uav_num_);
        has_com_.resize(uav_num_,false);
        has_new_com_.resize(uav_num_,false);
        last_seq_.resize(uav_num_,UINT32_MAX);
        data_timestamps_.resize(uav_num_);
        dist_error_.resize(uav_num_,0.0);
        angle_err_.resize(uav_num_);
        
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
    }
    void comCallback(const sensors::ComMsg::ConstPtr &msg, int uav_index)
    {
        geometry_msgs::Point Pc_j;
        Pc_j.x = msg->position.x + msg->velocity.x * DELTA_TIME;
        Pc_j.y = msg->position.y + msg->velocity.y * DELTA_TIME;
        Pc_j.z = msg->position.z + msg->velocity.z * DELTA_TIME;

        Pc[uav_index] = Pc_j;
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
        for (const auto &t : data_timestamps_)
        {
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
        for (const auto &t : data_timestamps_)
        {
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
                has_camera_ = true;
            }
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

    }
private:
    int uav_id_;
    int uav_num_;
    
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> com_subs_;
    ros::Subscriber ins_sub_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;

    std::vector<std::string> uavs_ = {"/iris_0","/iris_1","/iris_2","/iris_3"};
    std::vector<geometry_msgs::Point> Pc;
    std::vector<bool> has_new_com_;
    std::vector<bool> has_com_;
    std::vector<uint32_t> last_seq_;
    std::vector<ros::Time> data_timestamps_;   // 各无人机数据更新时间戳

    // UWB 缓存
    std::deque<std::pair<ros::Time, data_process::UwbProcessed>> uwb_history_;
    data_process::UwbProcessed best_uwb_;
    bool has_uwb_ = false;

    // CameraAngleMatch 缓存
    std::deque<std::pair<ros::Time, data_process::CameraAngleMatch>> camera_history_;
    data_process::CameraAngleMatch best_camera_;
    bool has_camera_ = false;

    

    //DGO
    ros::Timer DGO_Timer_;
    geometry_msgs::Point P_opt_;
    std::vector<double> dist_error_;    //距离误差
    bool dist_error_update_ = false;
    bool angle_error_update_ = false;
    struct AngleError
    {
        double alpha = 0.0;
        double theta = 0.0;
    };
    std::vector<AngleError> angle_err_; //角度误差, 按无人机 ID 索引

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
        return true;
    }
    /*DGO算法中误差函数和惩罚函数的计算*/
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

            double dx = Pc[target_id].x - P_opt_.x;
            double dy = Pc[target_id].y - P_opt_.y;
            double dz = Pc[target_id].z - P_opt_.z;

            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            double error = best_uwb_.distances[j] - dist;
            dist_error_[target_id] = error;
        }
        dist_error_update_ = true;
    }

    double cal_dist_penalty()
    {
        if(!dist_error_update_)
            return 0.0;

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
        dist_error_update_ = false;
        return penalty;
    }

    void cal_angle_error()
    {
        if(has_camera_)
        {
            //收到相机数据，开始计算角度误差
            const size_t n = std::min(int(best_camera_.count),uav_num_);
            for(int j = 0;j < n;j++)
            {
                int target_id = best_camera_.id[j];
                if(target_id < 0 || target_id >= uav_num_ || target_id == uav_id_)
                    continue;

                //根据P_opt和Pc[id]计算相对角度
                double dx = Pc[target_id].x - P_opt_.x;
                double dy = Pc[target_id].y - P_opt_.y;
                double dz = Pc[target_id].z - P_opt_.z;

                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-6)
                    continue;

                double alpha_error = normalizeAngle(best_camera_.alpha[j] - std::atan2(dy,dx));
                double theta_error = best_camera_.theta[j] - std::asin(dz / dist);

                angle_err_[target_id].alpha = alpha_error;
                angle_err_[target_id].theta = theta_error;
            }
            angle_error_update_ = true;
        }
    }

    double cal_angle_penalty()
    {
        if(!has_camera_ || !angle_error_update_)
            return 0.0;

        double penalty = 0.0;
        const size_t n = std::min({best_camera_.id.size(), best_camera_.alpha.size(), best_camera_.theta.size()});
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
        angle_error_update_ = false;
        return penalty;
    }
    static double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"DGO");
    DGO d;
    ros::spin();
    return 0;
}
