#ifndef GRIPBOT_KINEMATICS_CONTROL_HPP_
#define GRIPBOT_KINEMATICS_CONTROL_HPP_
#include <cstring>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <gripbot_sim_kinematics/SolvePositionIK.h>
#include <gripbot_sim_kinematics/kinematics.hpp>

namespace gripbot
{

class GripBotKinematicControl
{
public:
    // Public constructor
    GripBotKinematicControl(){};
    typedef boost::shared_ptr<GripBotKinematicControl> Ptr;
    static GripBotKinematicControl::Ptr create()
    {
        Ptr p_control(new GripBotKinematicControl());
        if (p_control->init())
        {
            return p_control;
        }
        return Ptr();
    }

    // Object initializer
    bool init();

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

private:
    /**
   * Callback function for the IK service that responds with the appropriate joint configuration or error message if not
   * found
   */
    bool IKCallback(
        gripbot_sim_kinematics::SolvePositionIK::Request &req,
        gripbot_sim_kinematics::SolvePositionIK::Response &res);

    // items
    bool init_controls;
    bool is_enabled;

    ros::ServiceServer m_ikService;
    ros::Subscriber joint_states_sub;
    ros::Subscriber robot_state_sub;
    ros::NodeHandle handle;
    sensor_msgs::JointState joint;

    GripBotKinematics::Ptr m_gripbotArm;
    std::vector<GripBotKinematics::Ptr> m_gripbotFingers;
};

} // namespace gripbot

#endif