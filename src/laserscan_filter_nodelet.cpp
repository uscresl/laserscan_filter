#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>

using namespace sensor_msgs;
using namespace message_filters;

namespace laserscan_filter_nodelet {
typedef sync_policies::ApproximateTime<LaserScan, Imu> SyncPolicy;

class LaserScanFilter : public nodelet::Nodelet {
public:
    LaserScanFilter() {}
    ~LaserScanFilter() {
        delete laser_sub_;
        delete imu_sub_;
        delete sync_;
    }

private:
    virtual void onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        
        private_nh.param<double>("roll_delta", roll_delta_, 0.034);
        private_nh.param<double>("pitch_delta", pitch_delta_, 0.034);
        
        laser_sub_ = new message_filters::Subscriber<LaserScan>(private_nh, "laser", 1);
        imu_sub_ = new message_filters::Subscriber<Imu>(private_nh, "imu", 1);
        sync_ = new Synchronizer<SyncPolicy>(SyncPolicy(10), *laser_sub_, *imu_sub_);
        sync_->registerCallback(boost::bind(&LaserScanFilter::callback, this, _1, _2));
        
        laser_pub_ = private_nh.advertise<LaserScan>("laserfiltered", 10);
        ROS_INFO("LaserScanFilterNodelet complete Init");
    }

    void callback(const LaserScan::ConstPtr &laser_scan, const Imu::ConstPtr &imu) {
        double roll, pitch, yaw;
        tf::Quaternion q(imu->orientation.x, imu->orientation.y,
                         imu->orientation.z, imu->orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        if (-roll_delta_ <= roll && roll <= roll_delta_ &&
                -pitch_delta_ <= pitch && pitch <= pitch_delta_) {
            laser_pub_.publish(laser_scan);
        }
    }
    ros::Publisher laser_pub_;
    message_filters::Subscriber<LaserScan> *laser_sub_;
    message_filters::Subscriber<Imu> *imu_sub_;
    Synchronizer<SyncPolicy> *sync_;
    
    double roll_delta_;
    double pitch_delta_;
};

PLUGINLIB_DECLARE_CLASS(laserscan_filter_nodelet, LaserScanFilter, laserscan_filter_nodelet::LaserScanFilter, nodelet::Nodelet);
}

