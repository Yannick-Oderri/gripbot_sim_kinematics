#include <gripbot_sim_kinematics/kinematics.hpp>
#include <gripbot_sim_kinematics/gripbot_kinematics_control.hpp>
#include <signal.h>

namespace gripbot
{
static const std::string JOINT_STATES = "/gripbot/joint_states";

bool GripBotKinematicControl::init()
{
    // default robot disable
    this->is_enabled = false;

    const std::string urdf_param("robot_description");
    int arm_joint_count = 0;

    this->m_gripbotArm = GripBotKinematics::create(
        urdf_param,
        "base_mount_link",
        "base_wrist_link",
        arm_joint_count);

    if (!this->m_gripbotArm)
    {
        ROS_FATAL_NAMED("gripbot", "Failed to load gripbot arm");
        return false;
    }

    std::string node_path = "/gripbot/kinematics/arm/pos";
    ros::NodeHandle arm_handle(node_path);

    // setup the service
    this->m_ikService = arm_handle.advertiseService(
        "IKService",
        &GripBotKinematicControl::IKCallback,
        this);

    return true;
}

/**
 * Callback function for the IK service that responds with the appropriate joint configuration or error message if not
 * found
 */
bool GripBotKinematicControl::IKCallback(gripbot_sim_kinematics::SolvePositionIK::Request &req,
                                     gripbot_sim_kinematics::SolvePositionIK::Response &res)
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

        if (!req.seed_angles.empty() && req.seed_mode != gripbot_sim_kinematics::SolvePositionIKRequest::SEED_CURRENT)
        {
            res.isValid[req_index] = this->m_gripbotArm->getPositionIK(
                req.pose_stamp[req_index], 
                req.seed_angles[req_index], 
                &joint_pose);

            res.joints[req_index].name.resize(joint_pose.name.size());
            res.result_type[req_index] = gripbot_sim_kinematics::SolvePositionIKRequest::SEED_USER;
            valid_inp = 1;
        }

        if ((!res.isValid[req_index]) && req.seed_mode != gripbot_sim_kinematics::SolvePositionIKRequest::SEED_USER)
        {
            res.isValid[req_index] = this->m_gripbotArm->getPositionIK(
                req.pose_stamp[req_index], joint, &joint_pose);
            res.joints[req_index].name.resize(joint_pose.name.size());
            res.result_type[req_index] = gripbot_sim_kinematics::SolvePositionIKRequest::SEED_CURRENT;
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
            res.result_type[req_index] = gripbot_sim_kinematics::SolvePositionIKResponse::RESULT_INVALID;
    }
    loop_rate.sleep();
}



} // namespace gripbot

std::shared_ptr<gripbot::GripBotKinematicControl> gripbot_kinematic_control;

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

/**
 * Entry point for program. Sets up Node, parses
 * command line arguments, then control loop (calling run() on Node)
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gripbot_kinematics", ros::init_options::NoSigintHandler);

    //capture signal and attemp to cleanup Node
    signal(SIGINT, quitRequested);

    gripbot_kinematic_control = gripbot::GripBotKinematicControl::create();

    if (gripbot_kinematic_control)
    {
        gripbot_kinematic_control->run();
    }

    return 0;
}