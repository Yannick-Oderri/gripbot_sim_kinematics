#include <gripbot_sim_kinematics/kinematics.hpp>
#include <gripbot_sim_kinematics/gripbot_arm_knmt_ctrl.hpp>

namespace gripbot
{
static const std::string JOINT_STATES = "/gripbot/joint_states";

bool gbKnmtCtrl::init()
{
    // default robot disable
    this->is_enabled = false;    

    const std::string urdf_param("robot_description");
    int arm_joint_count = 0;

    this->m_gripbotArm = GripBotKinematics::create(
        urdf_param,
        "base_mount_link",
        "forearm_visual_link",
        arm_joint_count);

    if (!this->m_gripbotArm)
    {
        ROS_FATAL_NAMED("gripbot", "Failed to load gripbot arm");
        return false;
    }

    std::string node_path = "/gripbot/kinematics/arm";
    ros::NodeHandle arm_handle(node_path);

    // setup the service
    this->m_ikService = arm_handle.advertiseService(
        "IKService",
        &gbKnmtCtrl::IKCallback,
        this);

    // Subscribe and advertise the subscribers and publishers accordingly for the Forward Kinematics
    joint_states_sub = handle.subscribe<sensor_msgs::JointState>(
        JOINT_STATES,
        100,
        &gbKnmtCtrl::FKCallback,
        this);
    end_pointstate_pub = handle.advertise<gripbot_core_msgs::EndpointState>(
        node_path + "/endpoint_state", 
        100);

    if (!handle.getParam("gripbot/arm_config/joint_names", joint_names))
    {
      ROS_FATAL_NAMED("gripbot", "GenericIK: No Joint Names for the Arm found on parameter server");
      return false;
    }

    return true;
}

/**
 * Callback function for the IK service that responds with the appropriate joint configuration or error message if not
 * found
 */
bool gbKnmtCtrl::IKCallback(
    gripbot_core_msgs::SolvePositionIK::Request &req,
    gripbot_core_msgs::SolvePositionIK::Response &res)
{
    ros::Rate loop_rate(100);
    sensor_msgs::JointState joint_pose;
    res.joints.resize(req.pose_stamp.size());
    res.isValid.resize(req.pose_stamp.size());
    res.result_type.resize(req.pose_stamp.size());
    for (size_t req_index = 0; req_index < req.pose_stamp.size(); req_index++)
    {
        res.isValid[req_index] = 0;
        int valid_inp = 0;

        if (!req.seed_angles.empty() && req.seed_mode != gripbot_core_msgs::SolvePositionIKRequest::SEED_CURRENT)
        {
            res.isValid[req_index] = this->m_gripbotArm->getPositionIK(
                req.pose_stamp[req_index],
                req.seed_angles[req_index],
                &joint_pose);

            res.joints[req_index].name.resize(joint_pose.name.size());
            res.result_type[req_index] = gripbot_core_msgs::SolvePositionIKRequest::SEED_USER;
            valid_inp = 1;
        }

        if ((!res.isValid[req_index]) && req.seed_mode != gripbot_core_msgs::SolvePositionIKRequest::SEED_USER)
        {
            res.isValid[req_index] = this->m_gripbotArm->getPositionIK(
                req.pose_stamp[req_index], joint, &joint_pose);
            res.joints[req_index].name.resize(joint_pose.name.size());
            res.result_type[req_index] = gripbot_core_msgs::SolvePositionIKRequest::SEED_CURRENT;
            valid_inp = 1;
        }

        if (!valid_inp)
        {
            ROS_ERROR_NAMED("position_kin", "Not a valid request message to the IK service");
            return false;
        }

        if (res.isValid[req_index])
        {
            res.joints[req_index].position.resize(joint_pose.position.size());
            res.joints[req_index].name = joint_pose.name;
            res.joints[req_index].position = joint_pose.position;
        }
        else
            res.result_type[req_index] = gripbot_core_msgs::SolvePositionIKResponse::RESULT_INVALID;
    }
    loop_rate.sleep();
}

/**
 * Callback function for the FK subscriber that retrievs the appropriate FK from the Joint states and publishes
 * to the endpoint_state topic
 */
void gbKnmtCtrl::FKCallback(const sensor_msgs::JointState msg)
{
    gripbot_core_msgs::EndpointState endpoint;

    sensor_msgs::JointState configuration;

    gbKnmtCtrl::FilterJointState(&msg, this->joint);
    // Copy the current Joint positions and names of the appropriate side to the configuration
    endpoint.pose = gbKnmtCtrl::FKCalc(this->joint).pose;

    // Fill out timestamp for endpoint
    endpoint.header.stamp = msg.header.stamp;

    // Publish the PoseStamp of the end effector
    end_pointstate_pub.publish(endpoint);
}

/**
 * Method to Filter the names and positions of the initialized side from the remaining
 */
void gbKnmtCtrl::FilterJointState(const sensor_msgs::JointState *msg, sensor_msgs::JointState &res)
{
    // Resize the result to hold the names and positions of the 7 joints
    res.name.resize(this->joint_names.size());
    res.position.resize(this->joint_names.size());
    int i = 0;
    for (size_t ind = 0; ind < msg->name.size(); ind++)
    {
        // Retain the names and positions of the joints of the initialized arm
        if (std::find(
            this->joint_names.begin(),
            this->joint_names.end(), 
            msg->name[ind]) != this->joint_names.end())
        {
            res.name[i] = msg->name[ind];
            res.position[i] = msg->position[ind];
            i++;
        }
    }
}

/**
 * Method to pass the desired configuration of the joints and calculate the FK
 * @return calculated FK
 */
geometry_msgs::PoseStamped gbKnmtCtrl::FKCalc(const sensor_msgs::JointState configuration)
{
    bool isV;
    geometry_msgs::PoseStamped fk_result;
    isV = this->m_gripbotArm->getPositionFK("base_mount_link", configuration, fk_result);
    return fk_result;
}

} // namespace gripbot

std::shared_ptr<gripbot::gbKnmtCtrl> gripbot_kinematic_control;

//! Helper function for
void quitRequested(int)
{
    if (gripbot_kinematic_control)
    {
        gripbot_kinematic_control->exit();
        gripbot_kinematic_control.reset();
    }

    ros::shutdown();
}
