#include <gripbot_sim_kinematics/gripbot_arm_knmt_ctrl.hpp>
#include <gripbot_sim_kinematics/gripbot_grpr_knmt_ctrl.hpp>
#include <gripbot_sim_kinematics/gripbot_ctrl.hpp>
#include <signal.h>

std::shared_ptr<gripbot::gbGripBotControl> pgripbot_control;

//! Helper function for
void quitRequested(int)
{
    if (pgripbot_control)
    {
        pgripbot_control->exit();
        // pgripbot_control->reset();
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

    pgripbot_control = std::shared_ptr<gripbot::gbGripBotControl>(new gripbot::gbGripBotControl());
    pgripbot_control->init();

    if (pgripbot_control)
    {
        pgripbot_control->run();
    }

    return 0;
}