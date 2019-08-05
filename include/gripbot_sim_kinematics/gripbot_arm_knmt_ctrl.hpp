#ifndef GRIPBOT_KINEMATICS_CONTROL_HPP_
#define GRIPBOT_KINEMATICS_CONTROL_HPP_
#include <cstring>
#include <ros/ros.h>
#include <gripbot_core_msgs/SolvePositionIK.h>
#include <gripbot_core_msgs/EndpointState.h>
#include <gripbot_sim_kinematics/kinematics.hpp>

namespace gripbot
{

class gbKnmtCtrl
{
public:
    // Public constructor
    gbKnmtCtrl(){};

    static std::shared_ptr<gbKnmtCtrl> create()
    {
        std::shared_ptr<gbKnmtCtrl> p_control(new gbKnmtCtrl());
        if (p_control->init())
        {
            return p_control;
        }
        return std::shared_ptr<gbKnmtCtrl>();
    }

    // Object initializer
    virtual bool init();

    /**
     * Method that serves as the main execution loop of the Node.  This is called in the main() function to 'run'
     * and only returns after exit or ros::shutdown is called.
    */
    void run()
    {
        // just do spin here (blocks until shutdown), remove while loop
        ros::spin();

        // we have left the ros spin loop, clean up (if needed) then shutdown
        exit();
    }

    /**
     * Method that allows signals (from their main function) to trigger any
     * cleanup and manually exit the node's run loop.
     * This is usually triggered by capturing a SIGTERM, etc.
    */
    void exit()
    {
        // Do anything to shut down cleanly
        // Note: Run loop will call shutdown before exiting

        this->m_ikService.shutdown();
        this->joint_states_sub.shutdown();
        this->robot_state_sub.shutdown();
        // this->end_pointstate_pub.shutdown();
    }

public:
    /**
     * Method to pass the desired configuration of the joints and calculate the FK
     * @return calculated FK pose
    */
    virtual geometry_msgs::PoseStamped FKCalc(const sensor_msgs::JointState req);

    /**
   * Callback function for the IK service that responds with the appropriate joint configuration or error message if not
   * found
   */
    virtual bool IKCallback(
        gripbot_core_msgs::SolvePositionIK::Request &req,
        gripbot_core_msgs::SolvePositionIK::Response &res);


    /**
     * Callback function for the FK subscriber that retrievs the appropriate FK from the Joint states and publishes it to
     * the endpoint
     * topic
    */
    virtual void FKCallback(const sensor_msgs::JointState msg);

    /**
     * Method to Filter the names and positions of the initialized side from the remaining
    */
    virtual void FilterJointState(const sensor_msgs::JointState* msg, sensor_msgs::JointState& res);  

protected:
    // items
    bool init_controls;
    bool is_enabled;

    ros::ServiceServer m_ikService;
    ros::Subscriber joint_states_sub;
    ros::Subscriber robot_state_sub;
    ros::Publisher end_pointstate_pub;
    ros::NodeHandle handle;
    sensor_msgs::JointState joint;
    std::vector<std::string>* joint_names;
    std::shared_ptr<GripBotKinematics> m_kinematicController;

};

} // namespace gripbot

#endif