#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_
#include <cstring>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/SetLinkProperties.h>
#include <gazebo_msgs/GetLinkProperties.h>
#include <algorithm>

namespace gripbot
{



typedef struct INFO
{
    std::vector<std::string> link_names;
    std::vector<std::string> joint_names;
} KinematicSolverInfo;

class GripBotKinematics
{
public:
    typedef boost::shared_ptr<GripBotKinematics> Ptr;
    static Ptr create(
        std::string urdf_param_name,
        std::string rootlink_name, 
        std::string endlink_name, int& no_jts)
    {
        Ptr p_kinematics = Ptr(new GripBotKinematics());
        if (p_kinematics->init(
            urdf_param_name,
            rootlink_name, 
            endlink_name, 
            no_jts))
        {
            return p_kinematics;
        }
        return Ptr();
    }

    GripBotKinematics();

    /**
     * Initializes kinematic element
     */
    bool init(std::string urdf, std::string root_name, std::string endlink_name, int &no_jts);

    /**
     * Method to calculate IK for end position
     */
    bool getPositionIK(const geometry_msgs::PoseStamped& pose_stamp, const sensor_msgs::JointState& seed,
                        sensor_msgs::JointState* result);

private:
    /* Method to load all the values from the parameter server
    *  @returns true is successful
    */
    bool loadModel(const std::string xml);

    /* Method to read the URDF model and extract the joints
    *  @returns true is successful
    */
    bool readJoints(urdf::Model& robot_model);

    /* Method to calculate the Joint index of a particular joint from the KDL chain
    *  @returns the index of the joint
    */
    int getJointIndex(const std::string& name);

    /* Method to calculate the KDL segment index of a particular segment from the KDL chain
    *  @returns the index of the segment
    */
    int getKDLSegmentIndex(const std::string& name);

    /* Members */
    ros::NodeHandle nh, nh_private;
    std::string root_name, endlink_name, joint_chain_name;
    KDL::JntArray joint_min, joint_max;
    KDL::Chain chain;
    unsigned int num_joints;

    KDL::ChainFkSolverPos_recursive* fk_solver; 
    KDL::ChainIkSolverPos_NR_JL* ik_solver_pos;
    KDL::ChainIkSolverVel_pinv* ik_solver_vel;
    

    ros::ServiceServer ik_server, ik_solver_info_service;

    tf::TransformListener tf_listener;
    std::vector<int> indd;
    std::vector<std::string> joints;

    KinematicSolverInfo info;
};


}

#endif