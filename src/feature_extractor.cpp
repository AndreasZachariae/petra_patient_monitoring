#include <petra_patient_monitoring/feature_extractor.h>

FeatureExtractor::FeatureExtractor() : Node("FeatureExtractor")
{
    patient_features_publisher_ = create_publisher<petra_interfaces::msg::PatientFeatures>("PatientFeatures", 10);

    pose_list_subscription_ = create_subscription<openpose_ros_msgs::msg::OpenPoseHumanList>("/openpose_ros/human_list", 10, [&](const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses) {
        RCLCPP_INFO(get_logger(), "Humans: %i", human_poses->num_humans);

        //returns -1 if patient is not detected
        int patient_idx = pt.get_patient_idx(human_poses);

        if (patient_idx >= 0)
        {
            RCLCPP_INFO(get_logger(), "Body: %i/25, Face: %i/70, Hand: %i/42",
                        human_poses->human_list.at(patient_idx).num_body_key_points_with_non_zero_prob,
                        human_poses->human_list.at(patient_idx).num_face_key_points_with_non_zero_prob,
                        human_poses->human_list.at(patient_idx).num_left_hand_key_points_with_non_zero_prob + human_poses->human_list.at(patient_idx).num_right_hand_key_points_with_non_zero_prob);

            publish_patient_features_(human_poses, patient_idx);

            //find_most_reliable_keypoint_(human_poses->human_list.at(patient_idx));
        }
    });
}

void FeatureExtractor::publish_patient_features_(const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses, int patient_idx)
{
    auto patient_pose = std::make_shared<openpose_ros_msgs::msg::OpenPoseHuman>(human_poses->human_list.at(patient_idx));

    openpose_ros_msgs::msg::BoundingBox torso_bb = calc_torso_bounding_box_(patient_pose);
    scale_buffer_.push(torso_bb.height);

    //height of torso boundig box averaged over last 100 frames
    //could also use predefined height of patient as scale
    double scale = scale_buffer_.get_weighted_sum().x;
    std::cout << "scale = " << scale << std::endl;

    petra_interfaces::msg::PatientFeatures patient_features;
    patient_features.header.stamp = now();
    patient_features.header.frame_id = "feature_extractor";
    patient_features.image_header = human_poses->image_header;

    //percentage of visible Keypoints
    //could use more data from presence sensors e.g. pulse
    patient_features.presence = patient_pose->num_body_key_points_with_non_zero_prob / (double)MAX_KEYPOINTS;

    //shape of the body without arm keypoints, width/height
    patient_features.torso_bounding_box_ratio = torso_bb.width / torso_bb.height;

    //scaled distance between head and ground (bottom edge of bounding box)
    patient_features.head_ground_distance = get_head_ground_distance_(patient_pose, scale);
    patient_features.buffered_head_ground_distance = head_ground_distance_buffer_.get_weighted_sum().x;

    //scaled height of torso bounding box (without arms), should be simmilar to head_ground_distance
    patient_features.torso_height = torso_bb.height / scale;
    height_buffer_.push(patient_features.torso_height);
    patient_features.buffered_torso_height = height_buffer_.get_weighted_sum().x;

    //scaled velocity of the head in y direction
    patient_features.head_y_velocity = calc_velocity_(human_poses, patient_idx, scale).y;
    patient_features.buffered_head_y_velocity = velocity_buffer_.get_weighted_sum().y;

    //scaled height of body centroid (without arms, legs, head)
    patient_features.centroid = calc_body_centroid_(patient_pose, scale).y;
    patient_features.buffered_centroid = centroid_buffer_.get_weighted_sum().y;

    patient_features_publisher_->publish(patient_features);
}

// finds bounding box around all Keypoints without the arms
openpose_ros_msgs::msg::BoundingBox FeatureExtractor::calc_torso_bounding_box_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose)
{
    std::vector<int> torso_keypoints = {0, 1, 2, 5, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

    //Pixel starten oben links im Bild mit (x=0,y=0)
    openpose_ros_msgs::msg::BoundingBox torso_bounding_box;
    double max_x, max_y = 0;
    bool first_loop = true;

    for (int k : torso_keypoints)
    {
        if (patient_pose->body_key_points_with_prob.at(k).prob > 0)
        {
            if (first_loop)
            {
                torso_bounding_box.x = patient_pose->body_key_points_with_prob.at(k).x;
                torso_bounding_box.y = patient_pose->body_key_points_with_prob.at(k).y;

                first_loop = false;
            }

            if (patient_pose->body_key_points_with_prob.at(k).x < torso_bounding_box.x)
            {
                torso_bounding_box.x = patient_pose->body_key_points_with_prob.at(k).x;
            }

            if (patient_pose->body_key_points_with_prob.at(k).y < torso_bounding_box.y)
            {
                torso_bounding_box.y = patient_pose->body_key_points_with_prob.at(k).y;
            }

            if (patient_pose->body_key_points_with_prob.at(k).x > max_x)
            {
                max_x = patient_pose->body_key_points_with_prob.at(k).x;
            }

            if (patient_pose->body_key_points_with_prob.at(k).y > max_y)
            {
                max_y = patient_pose->body_key_points_with_prob.at(k).y;
            }
        }
    }

    torso_bounding_box.height = max_y - torso_bounding_box.y;
    torso_bounding_box.width = max_x - torso_bounding_box.x;

    return torso_bounding_box;
}

// returns scaled distance between head and ground (bottom edge of bounding box)
double FeatureExtractor::get_head_ground_distance_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose, double scale)
{
    if (patient_pose->body_key_points_with_prob.at(0).prob > 0)
    {
        double hg_dist = (patient_pose->body_bounding_box.y + patient_pose->body_bounding_box.height - patient_pose->body_key_points_with_prob.at(0).y) / scale;

        head_ground_distance_buffer_.push(hg_dist);

        return hg_dist;
    }
    else
    {
        std::cout << "Head not detected" << std::endl;
        return -1;
    }
}

// calculates the velocity of the head from comparision with the last pose
Point FeatureExtractor::calc_velocity_(const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses, int patient_idx, double scale)
{
    if (human_poses->human_list.at(patient_idx).body_key_points_with_prob.at(0).prob > 0)
    {
        if (first_pose_)
        {
            first_pose_ = false;
            last_pose_ = human_poses;
            return Point();
        }

        Point velocity_vec;

        double time_passed = (human_poses->image_header.stamp.sec - last_pose_->image_header.stamp.sec) * 1000000000;
        time_passed += human_poses->image_header.stamp.nanosec - last_pose_->image_header.stamp.nanosec;

        std::cout << "Time passed [ms]= " << time_passed / 1000000 << " Fps= " << 1 / (time_passed / 1000000000) << std::endl;

        double delta_x = human_poses->human_list.at(patient_idx).body_key_points_with_prob.at(0).x - last_pose_->human_list.at(patient_idx).body_key_points_with_prob.at(0).x;
        double delta_y = human_poses->human_list.at(patient_idx).body_key_points_with_prob.at(0).y - last_pose_->human_list.at(patient_idx).body_key_points_with_prob.at(0).y;

        velocity_vec.x = ((delta_x / time_passed) * 1000000000) / scale; //in [1/s]
        velocity_vec.y = ((delta_y / time_passed) * 1000000000) / scale; //in [1/s]

        velocity_buffer_.push(velocity_vec);
        velocity_buffer_.push_weight(time_passed);

        std::cout << "vel_y [1/s]=" << velocity_vec.y << std::endl;

        std::cout << "buff_vel_y [1/s]=" << velocity_buffer_.get_weighted_sum().y << std::endl;

        last_pose_ = human_poses;

        return velocity_vec;
    }

    else
    {
        std::cout << "Head not detected" << std::endl;
        return Point();
    }
}

// PROBLEM: If one point is not detected anymore, the centroid jumps up/down
Point FeatureExtractor::calc_body_centroid_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose, double scale)
{
    std::vector<int> body_keypoints = {1, 2, 5, 8, 9, 12};

    Point centroid = Point();
    int num_of_keypoints = 0;
    double ground_y = patient_pose->body_bounding_box.y + patient_pose->body_bounding_box.height;

    for (int k : body_keypoints)
    {
        if (patient_pose->body_key_points_with_prob.at(k).prob > 0)
        {
            num_of_keypoints++;

            centroid.x += patient_pose->body_key_points_with_prob.at(k).x;
            centroid.y += patient_pose->body_key_points_with_prob.at(k).y;
        }
    }

    centroid.x = (centroid.x / num_of_keypoints) / scale;
    centroid.y = (ground_y - (centroid.y / num_of_keypoints)) / scale;

    centroid_buffer_.push(centroid);

    return centroid;
}

// collects meta-data about all Keypoints and averages over 100 frames
void FeatureExtractor::find_most_reliable_keypoint_(const openpose_ros_msgs::msg::OpenPoseHuman::SharedPtr patient_pose)
{
    for (size_t i = 0; i < patient_pose->body_key_points_with_prob.size(); i++)
    {

        if (patient_pose->body_key_points_with_prob.at(i).prob > 0)
        {
            keypoints_meta_data_.at(i).probability_buffer_.push(patient_pose->body_key_points_with_prob.at(i).prob);
            keypoints_meta_data_.at(i).availability_buffer_.push(1);
        }
        else
        {
            keypoints_meta_data_.at(i).availability_buffer_.push(0);
        }

        if (keypoints_meta_data_.at(i).availability_buffer_.get_weighted_sum().x > 0.5)
        {
            std::cout << "Keypoint: " << i
                      << " available: " << (int)(keypoints_meta_data_.at(i).availability_buffer_.get_weighted_sum().x * 100)
                      //<< "% avg_prob: " << keypoints_meta_data_.at(i).probability_buffer_.get_weighted_sum().x
                      << "% avg_prob: " << keypoints_meta_data_.at(i).probability_buffer_.get_geom_mean().x << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureExtractor>());
    rclcpp::shutdown();

    return 0;
}