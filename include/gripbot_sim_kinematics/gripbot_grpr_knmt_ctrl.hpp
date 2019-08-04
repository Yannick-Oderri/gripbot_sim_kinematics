#ifndef GRIPBOT_GRIPPER_CONTROL_HPP_
#define GRIPBOT_GRIPPER_CONTROL_HPP_

#include <cstring>
#include <ros/ros.h>
#include <gripbot_sim_kinematics/kinematics.hpp>


namespace gripbot
{

class GripBotGripperKinematics
{
public:
    bool init();
    


private:
    ros::NodeHandle controllerNode;
    GripBotKinematics kinematicController;
    std::string name;
    std::string rootlink_name;
    std::string endlink_name;
};


}




#endif