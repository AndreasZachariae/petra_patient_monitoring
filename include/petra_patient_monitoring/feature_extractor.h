/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : petra_patient_monitoring
 * Purpose : Extracts features from the OpenPose output and publishes them for further processing
 *
 * @author Andreas Zachariae
 * @since 1.1.0 (2020.11.30)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <openpose_ros_msgs/msg/bounding_box.hpp>
#include <openpose_ros_msgs/msg/open_pose_human.hpp>
#include <openpose_ros_msgs/msg/open_pose_human_list.hpp>
#include <openpose_ros_msgs/msg/point_with_prob.hpp>

#include <petra_interfaces/msg/patient_features.hpp>
#include <petra_patient_monitoring/buffer.h>
#include <petra_patient_monitoring/patient_tracker.h>

int MAX_KEYPOINTS = 25;

struct meta_data
{
    buffer probability_buffer_ = buffer(100);
    buffer availability_buffer_ = buffer(100);
};

class FeatureExtractor : public rclcpp::Node
{
public:
    FeatureExtractor();

private:
    rclcpp::Subscription<openpose_ros_msgs::msg::OpenPoseHumanList>::SharedPtr pose_list_subscription_;
    rclcpp::Publisher<petra_interfaces::msg::PatientFeatures>::SharedPtr patient_features_publisher_;

    patient_tracker pt;

    buffer scale_buffer_ = buffer(100);
    buffer head_ground_distance_buffer_ = buffer(5);
    buffer height_buffer_ = buffer(5);
    buffer velocity_buffer_ = buffer(5);
    buffer centroid_buffer_ = buffer(5);

    openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr last_pose_;
    bool first_pose_ = true;

    std::vector<meta_data> keypoints_meta_data_ = std::vector<meta_data>(25);

    void publish_patient_features_(const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses, int patient_idx);

    Point calc_body_centroid_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose, double scale);

    openpose_ros_msgs::msg::BoundingBox calc_torso_bounding_box_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose);

    double get_head_ground_distance_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose, double scale);

    Point calc_velocity_(const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses, int patient_idx, double scale);

    void find_most_reliable_keypoint_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose);
};