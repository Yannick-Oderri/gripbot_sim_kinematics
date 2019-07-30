#include <gripbot_sim_kinematics/kinematics.hpp>

namespace gripbot
{
GripBotKinematics::GripBotKinematics() : nh_private("~")
{
}

bool GripBotKinematics::init(
    std::string urdf_param_name,
    std::string rootLink, 
    std::string endlink, int &no_jts)
{
    std::string full_urdf_xml = urdf_param_name;
    this->root_name = rootLink;
    this->endlink_name = endlink;
    // this->nh.param("gripbot_urdf_xml", urdf_xml, std::string("gripbot_description"));
    // this->nh.searchParam(urdf_xml, full_urdf_xml);
    ROS_DEBUG_NAMED("gripbot_kinematics", "Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result))
    {
        ROS_FATAL_NAMED("gripbot", "Could not load xml from parameter server: %s", urdf_param_name.c_str());
        return false;
    }

    if (!this->loadModel(result))
    {
        ROS_FATAL_NAMED("gripbot_kinematics", "Could not load Model!");
        return false;
    }

    // Get Solver parameters
    int maxIterations;
    double epsilon;

    // Get root and tim from parameter service
    this->nh_private.param("maxIterations", maxIterations, 1000);
    nh_private.param("epsilon", epsilon, 1e-2);

    // Build Solver
    this->fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    this->ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
    this->ik_solver_pos =
        new KDL::ChainIkSolverPos_NR_JL(
            this->chain,
            this->joint_min,
            this->joint_max,
            *this->fk_solver,
            *this->ik_solver_vel,
            maxIterations,
            epsilon);
    no_jts = this->num_joints;
}

bool GripBotKinematics::loadModel(const std::string xml)
{
    urdf::Model robot_model;
    KDL::Tree ktree;
    if (!robot_model.initString(xml))
    {
        ROS_FATAL_NAMED("gripbot_kinematics", "Could not initialize robot model");
        return false;
    }
    if (!kdl_parser::treeFromString(xml, ktree))
    {
        ROS_ERROR_NAMED("gripbot_kinematics", "Could not initialize kdl tree.");
        return false;
    }
    if (!ktree.getChain(this->root_name, this->endlink_name, chain))
    {
        ROS_ERROR_NAMED("gripbot_kinematics",
                        "Could not initialize chain object for root_name %s and and endlink_name %s",
                        root_name.c_str(), endlink_name.c_str());
        return false;
    }
    if (!this->readJoints(robot_model))
    {
        ROS_FATAL_NAMED("gripbot", "Could not read information about the joints");
    }

    return true;
}

bool GripBotKinematics::readJoints(urdf::Model &robot_model)
{
    this->num_joints = 0;
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(endlink_name);
    boost::shared_ptr<const urdf::Joint> joint;

    for (int i = 0; i < chain.getNrOfSegments(); i++)
        while (link && link->name != root_name)
        {
            if (!(link->parent_joint))
            {
                break;
            }
            joint = robot_model.getJoint(link->parent_joint->name);
            if (!joint)
            {
                ROS_ERROR_NAMED("gripbot_kinematics", "Could not find joint: %s", link->parent_joint->name.c_str());
                return false;
            }
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {
                ROS_DEBUG_NAMED("gripbot_kinematics", "adding joint: [%s]", joint->name.c_str());
                num_joints++;
            }
            link = robot_model.getLink(link->getParent()->name);
        }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.link_names.resize(num_joints);

    link = robot_model.getLink(endlink_name);
    unsigned int i = 0;
    while (link && i < num_joints)
    {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
        {
            ROS_DEBUG_NAMED("gripbot_kinematics", "getting bounds for joint: [%s]", joint->name.c_str());

            float lower, upper;
            int hasLimits;
            if (joint->type != urdf::Joint::CONTINUOUS)
            {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            }
            else
            {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i - 1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.link_names[index] = link->name;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}

/* Method to calculate the Joint index of a particular joint from the KDL chain
 *  @returns the index of the joint
 */
int GripBotKinematics::getJointIndex(const std::string &name)
{
    for (unsigned int i = 0; i < this->info.joint_names.size(); i++)
    {
        if (this->info.joint_names[i] == name)
            return i;
    }
    return -1;
}

/* Method to calculate the KDL segment index of a particular segment from the KDL chain
 *  @returns the index of the segment
 */
int GripBotKinematics::getKDLSegmentIndex(const std::string &name)
{
    int i = 0;
    while (i < (int)chain.getNrOfSegments())
    {
        if (chain.getSegment(i).getJoint().getName() == name)
        {
            return i + 1;
        }
        i++;
    }
    return -1;
}

/* Method to calculate the IK for the required end pose
 *  @returns true if successful
 */
bool GripBotKinematics::getPositionIK(const geometry_msgs::PoseStamped &pose_stamp,
                                               const sensor_msgs::JointState &seed, sensor_msgs::JointState *result)
{
    geometry_msgs::PoseStamped pose_msg_in = pose_stamp;
    tf::Stamped<tf::Pose> transform;
    tf::Stamped<tf::Pose> transform_root;
    tf::poseStampedMsgToTF(pose_msg_in, transform);

    // Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;

    jnt_pos_in.resize(this->num_joints);
    // Copying the positions of the joints relative to its index in the KDL chain
    for (unsigned int i = 0; i < this->num_joints; i++)
    {
        int tmp_index = this->getJointIndex(seed.name[i]);
        if (tmp_index >= 0)
        {
            jnt_pos_in(tmp_index) = seed.position[i];
        }
        else
        {
            ROS_ERROR_NAMED("arm_kinematics", "i: %d, No joint index for %s", i, seed.name[i].c_str());
        }
    }

    // Convert F to our root_frame
    try
    {
        this->tf_listener.transformPose(this->root_name, transform, transform_root);
    }
    catch (...)
    {
        ROS_ERROR_NAMED("gripbot_kinematics", "Could not transform IK pose to frame: %s", root_name.c_str());
        return false;
    }

    KDL::Frame F_dest;
    tf::transformTFToKDL(transform_root, F_dest);

    int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

    if (ik_valid >= 0)
    {
        result->name = info.joint_names;
        result->position.resize(num_joints);
        for (unsigned int i = 0; i < num_joints; i++)
        {
            result->position[i] = jnt_pos_out(i);
            ROS_DEBUG_NAMED("arm_kinematics", "IK Solution: %s %d: %f", result->name[i].c_str(), i, jnt_pos_out(i));
        }
        return true;
    }
    else
    {
        ROS_DEBUG_NAMED("arm_kinematics", "An IK solution could not be found");
        return false;
    }
}

} // namespace gripbot