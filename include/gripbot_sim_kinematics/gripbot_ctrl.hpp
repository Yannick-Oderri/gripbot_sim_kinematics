#include <gripbot_sim_kinematics/gripbot_grpr_knmt_ctrl.hpp>
#include <boost/format.hpp>

namespace gripbot
{
    typedef gbKnmtCtrl gbArmKnmtCtrl;
    typedef std::shared_ptr<gbKnmtCtrl> pgbArmKnmtCtrl;

    class gbGripBotControl
    {
        public:
        gbGripBotControl():
                m_effector_ctrl_list(3),
                m_arm_ctrl(){};


        bool init()
        {
            // initialize arm kinematics
            if(!this->m_arm_ctrl.init()){
                ROS_FATAL_NAMED("gripbot", "Error while initialize gripbot arm.");
                return false;
            }
            

            for(std::vector<gbGrprKnmtCtrl>::size_type i = 0; i != this->m_effector_ctrl_list.size(); i++) {
                
                std::string effector_name = "finger_" + std::to_string(i + 1);
                std::string rootlink_name = effector_name + "_base_link";
                std::string endlink_name = effector_name + "_distal_link";
                gbGrprKnmtCtrl& effector = this->m_effector_ctrl_list[i];

                if (i == 2)
                    rootlink_name = effector_name + "_proximal_link";
                if (!effector.init(effector_name, rootlink_name, endlink_name)){
                    ROS_FATAL_NAMED("gripbot", "Error while initializing gripbot end effector.");
                    return false;
                }
            }

            return true;
        }

        gbArmKnmtCtrl& getArmControls(){
            return this->m_arm_ctrl;
        }

        gbGrprKnmtCtrl& getGripperControls(int index){
            return this->m_effector_ctrl_list[index];
        }



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
            this->m_arm_ctrl.exit();
            for(auto& effector : this->m_effector_ctrl_list){
                effector.exit();
            }
        }

        protected:
        std::vector<gbGrprKnmtCtrl> m_effector_ctrl_list;
        gbArmKnmtCtrl m_arm_ctrl;
        

    };

    
}