#ifndef STELLA_SLAM_ROS_H
#define STELLA_SLAM_ROS_H

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/stereo_rectifier.h>


#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "tf2/LinearMath/Transform.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <std_srvs/srv/empty.hpp>
#include <stella_vslam_ros/msg/stella_vslam_status.hpp>
namespace stella_vslam_ros {


// Transformation converting from
// Canonical ROS Frame (x-forward, y-left, z-up) to
// cuVSLAM Frame       (x-right, y-up, z-backward)
// ROS    ->  cuVSLAM
//  x     ->    -z
//  y     ->    -x
//  z     ->     y
const tf2::Transform cuvslam_pose_canonical(tf2::Matrix3x3(
    0, -1, 0,
    0, 0, 1,
    -1, 0, 0
));
// Transformation converting from
// cuVSLAM Frame       (x-right, y-up, z-backward) to
// Canonical ROS Frame (x-forward, y-left, z-up)
const tf2::Transform canonical_pose_cuvslam(cuvslam_pose_canonical.inverse());
// Transformation converting from
// Optical Frame    (x-right, y-down, z-forward) to
// cuVSLAM Frame    (x-right, y-up, z-backward)
// Optical   ->  cuVSLAM
//    x      ->     x
//    y      ->    -y
//    z      ->    -z
const tf2::Transform cuvslam_pose_optical(tf2::Matrix3x3(
    1, 0, 0,
    0, -1, 0,
    0, 0, -1
));
// Transformation converting from
// cuVSLAM Frame    (x-right, y-up, z-backward) to
// Optical Frame    (x-right, y-down, z-forward)
const tf2::Transform optical_pose_cuvslam(cuvslam_pose_optical.inverse());
class system {
public:
    system(const std::shared_ptr<stella_vslam::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path);
    void publish_pose(const std::shared_ptr<Eigen::Matrix4d> cam_pose_wc, const rclcpp::Time& stamp);
    void publish_keyframes(const rclcpp::Time& stamp);
    void publish_landmarks(const rclcpp::Time& stamp);
    void publish_status(const rclcpp::Time& stamp) ;
    void setParams();
    void capture_image(const cv::Mat &img);
    std::shared_ptr<stella_vslam::system> slam_;
    std::shared_ptr<stella_vslam::config> cfg_;
    rclcpp::Node* node_;
    rmw_qos_profile_t custom_qos_;
    cv::Mat mask_;
    geometry_msgs::msg::Pose last_pose_; // last computed pose
    std::vector<double> track_times_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> keyframes_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> keyframes_2d_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_pub_;
    std::shared_ptr<rclcpp::Publisher<stella_vslam_ros::msg::StellaVslamStatus>> status_pub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
        init_pose_sub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> wheel_odom_sub_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> reset_srv_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string robot_base_frame_;
    std::string camera_frame_;
    std::string camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    // camera to baselink transform (usually provided by URDF)
    tf2::Transform camera_to_base_link_;

    // If true the camera is mounted facing backwards
    // the camera pose & keyframes will be rotated 180 degrees around the Z axis
    bool camera_backwards_;

    // If true will capture images
    bool img_capture_;
    bool img_overlay_stats_; // will overlay stats on the image

    // If true will publish the status of the SLAM system
    bool publish_status_;

    // If true, publish tf from map_frame to odom_frame
    bool publish_tf_;

    // If true, publish keyframes
    bool publish_keyframes_;

    // Publish pose's timestamp in the future
    double transform_tolerance_;

    // If true, odom_frame is fixed on the xy-plane of map_frame. This is useful when working with 2D navigation modules.
    bool odom2d_;

    std::string encoding_;

    // Support functions to get Transform fro Odometry message
    Eigen::Affine3d convertOdometryToEigenAffine3d(const nav_msgs::msg::Odometry& odom) {
        // Extract position from the odometry message
        const auto& position = odom.pose.pose.position;
        Eigen::Vector3d eigen_position(position.x, position.y, position.z);

        // Extract orientation from the odometry message
        const auto& orientation = odom.pose.pose.orientation;
        Eigen::Quaterniond eigen_quaternion(orientation.w, orientation.x, orientation.y, orientation.z);

        // Construct the Eigen::Affine3d object using position and orientation
        Eigen::Affine3d eigen_transform = Eigen::Affine3d::Identity();
        eigen_transform.prerotate(eigen_quaternion);
        eigen_transform.pretranslate(eigen_position);

        return eigen_transform;
    }

    // Function to convert tf2::Transform to Eigen::Affine3d
    Eigen::Affine3d tf2TransformToEigenAffine3d(const tf2::Transform& tf_transform) {
        // Extract the rotation from tf2 Transform
        tf2::Quaternion tf_quaternion = tf_transform.getRotation();
        Eigen::Quaterniond eigen_quaternion(tf_quaternion.getW(), tf_quaternion.getX(), tf_quaternion.getY(), tf_quaternion.getZ());

        // Extract the translation from tf2 Transform
        tf2::Vector3 tf_translation = tf_transform.getOrigin();
        Eigen::Translation3d eigen_translation(tf_translation.getX(), tf_translation.getY(), tf_translation.getZ());

        // Create an Eigen Affine3d transform combining rotation and translation
        Eigen::Affine3d eigen_transform = Eigen::Affine3d(eigen_translation * eigen_quaternion);

        return eigen_transform;
    }
    
    inline tf2::Transform poseToTransform(const geometry_msgs::msg::Pose &pose) {
        tf2::Quaternion quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
        return tf2::Transform(quaternion, translation);
    }

    inline geometry_msgs::msg::Pose transformToPose(const tf2::Transform &transform) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();
        pose.orientation.x = transform.getRotation().x();
        pose.orientation.y = transform.getRotation().y();
        pose.orientation.z = transform.getRotation().z();
        pose.orientation.w = transform.getRotation().w();
        return pose;
    }
    // debug timer
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex timer_lock_;
    void timer_callback();
    // timer tracks
    bool image_received_ {false};

private:
    Eigen::AngleAxisd rot_ros_to_cv_map_frame_;

    // Wheel odometry related variables
    nav_msgs::msg::Odometry  wheelOdom_;
    std::mutex       wheelOdom_lock_;
    bool             wheelOdom_initialized_;

    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    // Callback function for wheel odometry
    void RecordWheelOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg_wheel_odom);
    // Returns the wheel Odometry Transform
    nav_msgs::msg::Odometry getWheelOdom() ;

    // Capture image related variables
    float img_capture_distance_thr_;
    float img_capture_angle_thr_;
    std::string img_capture_path_;
};

class mono : public system {
public:
    mono(const std::shared_ptr<stella_vslam::system>& slam,
         rclcpp::Node* node,
         const std::string& mask_img_path);
    void callback(sensor_msgs::msg::Image::UniquePtr msg);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> raw_image_sub_;
};

template<class M, class NodeType = rclcpp::Node>
class ModifiedSubscriber : public message_filters::Subscriber<M> {
public:
    ModifiedSubscriber(typename message_filters::Subscriber<M>::NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_services_default)
        : message_filters::Subscriber<M>(node, topic, qos) {
    }
    ModifiedSubscriber(NodeType* node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_services_default)
        : message_filters::Subscriber<M>(node, topic, qos) {
    }
    ModifiedSubscriber(
        typename message_filters::Subscriber<M>::NodePtr node,
        const std::string& topic,
        const rmw_qos_profile_t qos,
        rclcpp::SubscriptionOptions options)
        : message_filters::Subscriber<M>(node, topic, qos, options) {
    }
    ModifiedSubscriber(
        NodeType* node,
        const std::string& topic,
        const rmw_qos_profile_t qos,
        rclcpp::SubscriptionOptions options)
        : message_filters::Subscriber<M>(node, topic, qos, options) {
    }
    void cb(const typename message_filters::Subscriber<M>::MConstPtr& msg) {
        this->signalMessage(msg);
    }
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<stella_vslam::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path,
           const std::shared_ptr<stella_vslam::util::stereo_rectifier>& rectifier);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right);

    std::shared_ptr<stella_vslam::util::stereo_rectifier> rectifier_;
    ModifiedSubscriber<sensor_msgs::msg::Image> left_sf_, right_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<stella_vslam::system>& slam,
         rclcpp::Node* node,
         const std::string& mask_img_path);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    ModifiedSubscriber<sensor_msgs::msg::Image> color_sf_, depth_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

} // namespace stella_vslam_ros

#endif // STELLA_SLAM_ROS_H
