class patient_tracker
{
private:
    int patient_idx_ = 0;

public:
    patient_tracker(/* args */);

    int get_patient_idx(const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses);
};

patient_tracker::patient_tracker(/* args */)
{
}

//returns -1 if patient could not be detected
int patient_tracker::get_patient_idx(const openpose_ros_msgs::msg::OpenPoseHumanList::SharedPtr human_poses)
{
    if (human_poses->num_humans > 0)
    {
        return patient_idx_;
    }
    else
    {
        return -1;
    }
}