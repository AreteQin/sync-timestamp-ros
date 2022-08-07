//
// Created by qin on 5/14/22.
//
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sophus/se3.hpp>

// sync sensor data
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>


void Callback(const geometry_msgs::QuaternionStampedConstPtr &q, const geometry_msgs::PointStampedConstPtr &t) {
    if ((q->quaternion.x == 0 && q->quaternion.y == 0 && q->quaternion.z == 0 && q->quaternion.w == 0)) {
        return;
    }
    Sophus::SE3f pose_se3;
    Eigen::Quaternion<float> quaternion;
    Eigen::Matrix<float, 3, 1> translation;
    translation.x() = t->point.x;
    translation.y() = t->point.y;
    translation.z() = t->point.z;
    quaternion.w() = q->quaternion.w;
    quaternion.x() = q->quaternion.x;
    quaternion.y() = q->quaternion.y;
    quaternion.z() = q->quaternion.z;
    pose_se3 = Sophus::SE3f(quaternion, translation);
    std::cout << "current position: " << std::endl << pose_se3.matrix() << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_subscriber");
    ros::NodeHandle nh_pose;
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> sub_quaternion;
    sub_quaternion.subscribe(nh_pose, "camera/pose/quaternion", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_translation;
    sub_translation.subscribe(nh_pose, "camera/pose/translation", 1);
    //将两个话题的数据进行同步
//    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::QuaternionStamped, geometry_msgs::PointStamped> syncPolocy;
    message_filters::TimeSynchronizer<geometry_msgs::QuaternionStamped, geometry_msgs::PointStamped> sync(sub_quaternion, sub_translation, 10);
    //指定一个回调函数, 实现两个数据的同步读取
    sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::Rate rate(30.0);
    while (nh_pose.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}