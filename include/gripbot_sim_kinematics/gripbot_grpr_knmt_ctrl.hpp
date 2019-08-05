#ifndef GRIPBOT_GRIPPER_CONTROL_HPP_
#define GRIPBOT_GRIPPER_CONTROL_HPP_

#include <cstring>
#include <ros/ros.h>
#include <gripbot_core_msgs/SolvePositionIK.h>
#include <gripbot_core_msgs/EndpointState.h>
#include <gripbot_sim_kinematics/gripbot_arm_knmt_ctrl.hpp>

namespace gripbot
{

    class gbGrprKnmtCtrl: public gbKnmtCtrl
    {
    public:
        virtual bool init(
            std::string name,
            std::string rootlink_name,
            std::string endlink_name
        );
        
    protected:


        bool is_enabled;
        ros::ServiceServer m_ikService;
        ros::Subscriber m_joint_states_sub;
        ros::Publisher m_endlink_pub;
        ros::NodeHandle m_controllerNode;
        std::string m_name;
        std::string m_robotlink_name;
        std::string m_endlink_name;
    };


}




#endif