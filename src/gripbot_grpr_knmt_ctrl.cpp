#include <gripbot_sim_kinematics/gripbot_grpr_knmt_ctrl.hpp>


using namespace gripbot;

const std::string JOINT_STATES("/gripbot/joint_states");


bool gbGrprKnmtCtrl::init(
    std::string name,
    std::string rootlink_name,
    std::string endlink_name
){
    this->m_name = name;
    this->is_enabled = false;    
    int joint_count = 0;

    const std::string urdf_param("robot_description");
    this->m_kinematicController = GripBotKinematics::create(
        urdf_param,
        rootlink_name,
        endlink_name,
        joint_count
    );

    if (!this->m_kinematicController){
        ROS_FATAL_NAMED("gripbot", "Faield to load Gripbot");
        return false;
    }

    std::string node_path("/gripbot/kinematics/gripper/" + this->m_name);
    ros::NodeHandle effector_handle(node_path);

    gbKnmtCtrl * ctrl = this;
    this->m_ikService = effector_handle.advertiseService(
        "ikservice",
        &gbKnmtCtrl::IKCallback,
        ctrl
    );

    // Subscribe and advertise the subscribers and publishers accordingly for the Forward Kinematics
    this->m_joint_states_sub = this->m_controllerNode.subscribe<sensor_msgs::JointState>(
        "/gripbot/joints_states",
        100,
        &gbKnmtCtrl::FKCallback,
        ctrl);

    this->m_endlink_pub = this->m_controllerNode.advertise<gripbot_core_msgs::EndpointState>(
        node_path + "/endpoint_state", 
        100);

    return true;

}
