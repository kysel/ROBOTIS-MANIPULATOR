
#ifndef MANIPULATOR_BASEMODULE_ROBOTISSTATE_H_
#define MANIPULATOR_BASEMODULE_ROBOTISSTATE_H_

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "robotis_math/RobotisMath.h"
#include "manipulator_kinematics_dynamics/ManipulatorKinematicsDynamics.h"

#include "manipulator_base_module_msgs/JointPose.h"
#include "manipulator_base_module_msgs/KinematicsPose.h"

namespace ROBOTIS_BASE
{

class RobotisState
{
public:

    RobotisState();
    ~RobotisState();

    bool is_moving;

    // trajectory
    int cnt;
    double mov_time;
    double smp_time;
    int all_time_steps;

    Eigen::MatrixXd calc_joint_tra;
    Eigen::MatrixXd calc_task_tra;

    Eigen::MatrixXd joint_ini_pose;

    // msgs
    manipulator_base_module_msgs::JointPose joint_pose_msg;
    manipulator_base_module_msgs::KinematicsPose kinematics_pose_msg;

    // inverse kinematics
    bool ik_solve;
    Eigen::MatrixXd ik_target_position;
    Eigen::MatrixXd ik_start_rotation , ik_target_rotation;
    int ik_id_start , ik_id_end;

    void setInverseKinematics(int cnt , Eigen::MatrixXd start_rotation);
};

}

#endif /* MANIPULATOR_BASEMODULE_ROBOTISSTATE_H_ */
