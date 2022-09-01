#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sophus/se3.hpp>
#include <clocale>

int main(int argc, char **argv) {

    // open camera
    cv::VideoCapture capture(0); //read the video from camera
    if (!capture.isOpened()) {
        ROS_ERROR_STREAM("Failed to open video device\n");
        ros::shutdown();
    }

    // ROS initialization
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh_boardcaster;
    tf2_ros::TransformBroadcaster pose_br;
    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(nh_boardcaster);
    image_transport::Publisher pub_color = it.advertise("camera/color/image_raw", 1);
    image_transport::Publisher pub_depth = it.advertise("camera/depth/image_raw", 1);
    ros::Publisher pub_quaternion = nh_boardcaster.advertise<geometry_msgs::QuaternionStamped>("camera/pose/quaternion",
                                                                                               1);
    geometry_msgs::QuaternionStamped msg_quaternion;
    ros::Publisher pub_translation = nh_boardcaster.advertise<geometry_msgs::PointStamped>("camera/pose/translation",
                                                                                           1);
    geometry_msgs::PointStamped msg_translation;
    double x = 0.0;
    cv::Mat rgb, depth;

    while (ros::ok()) {

        // simulate a SE3 pose
        Eigen::Matrix3f R = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f(0, 0, 1)).toRotationMatrix();
        Eigen::Vector3f t(x, 0, 0);
        x = x + 0.0001;
        Sophus::SE3f current_position(R, t);

        // initial translation and rotation publisher
        msg_quaternion.header.stamp = ros::Time::now();
        msg_quaternion.quaternion.x = current_position.unit_quaternion().x();
        msg_quaternion.quaternion.y = current_position.unit_quaternion().y();
        msg_quaternion.quaternion.z = current_position.unit_quaternion().z();
        msg_quaternion.quaternion.w = current_position.unit_quaternion().w();
        pub_quaternion.publish(msg_quaternion);
        msg_translation.header.stamp = msg_quaternion.header.stamp;
        msg_translation.point.x = current_position.translation()[0];
        msg_translation.point.y = current_position.translation()[1];
        msg_translation.point.z = current_position.translation()[2];
        pub_translation.publish(msg_translation);

        // Get the current RGB-D frame's RGB and depth images. This may wait for I/O
        // to complete in case it did not complete in the pre-loading thread yet.
        capture >> rgb; //load
        if (rgb.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown();
        }
        cv::cvtColor(rgb, depth, cv::COLOR_BGR2GRAY);

        // initial rgbd images publisher
        pub_color.publish(cv_bridge::CvImage(msg_quaternion.header, "rgb8", rgb).toImageMsg());
        pub_depth.publish(cv_bridge::CvImage(msg_quaternion.header, "mono8", depth).toImageMsg());

        ros::spinOnce();
    }
}
