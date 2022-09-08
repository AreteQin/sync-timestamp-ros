//
// Created by qin on 5/2/22.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <tf/transform_listener.h>
#include <sophus/se3.hpp>

// sync sensor data
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// sync image
#include <image_transport/subscriber_filter.h>

void Callback(const geometry_msgs::QuaternionStampedConstPtr &q, const geometry_msgs::PointStampedConstPtr &t,
              const sensor_msgs::ImageConstPtr &color, const sensor_msgs::ImageConstPtr &depth) {
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
    try {
        cv::imshow("camera/depth/image_raw", cv_bridge::toCvShare(depth, "mono16")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", depth->encoding.c_str());
    }
    try {
        cv::imshow("camera/color/image_raw", cv_bridge::toCvShare(color, "rgb8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", color->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_subscriber");
    ros::NodeHandle nh_loop_detector;
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> sub_quaternion;
    sub_quaternion.subscribe(nh_loop_detector, "camera/pose/quaternion", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_translation;
    sub_translation.subscribe(nh_loop_detector, "camera/pose/translation", 1);

    cv::namedWindow("camera/color/image_raw");
    cv::namedWindow("camera/depth/image_raw");
    image_transport::ImageTransport it(nh_loop_detector);
    image_transport::SubscriberFilter sub_color(it, "camera/color/image_raw", 1);
    image_transport::SubscriberFilter sub_depth(it, "camera/depth/image_raw", 1);

    //将话题的数据进行同步
    // Sync the subscribed data
    message_filters::TimeSynchronizer<geometry_msgs::QuaternionStamped,
                                      geometry_msgs::PointStamped,
                                      sensor_msgs::Image, sensor_msgs::Image>
            sync(sub_quaternion, sub_translation, sub_color, sub_depth, 10);
    //所有subscriber使用一个回调函数，_1至_4分别是callback的四个输入参数
    // All subscribers use one callback function, _1 to _4 is the input variables
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3, _4));

    ros::Rate rate(30.0);
    while (nh_loop_detector.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}