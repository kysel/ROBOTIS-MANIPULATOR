/*
 * DemoModule.h
 *
 *  Created on: 2016. 3. 9.
 *      Author: sch
 */

#ifndef MANIPULATOR_BASEMODULE_H_
#define MANIPULATOR_BASEMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/MotionModule.h"
#include "robotis_math/RobotisMath.h"
#include "manipulator_kinematics_dynamics/ManipulatorKinematicsDynamics.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "RobotisState.h"

#include "manipulator_base_module_msgs/JointPose.h"
#include "manipulator_base_module_msgs/KinematicsPose.h"

#include "manipulator_base_module_msgs/GetJointPose.h"
#include "manipulator_base_module_msgs/GetKinematicsPose.h"

namespace ROBOTIS
{

//using namespace ROBOTIS_DEMO;

class BaseJointData
{

public:
    double position;
    double velocity;
    double effort;

    int p_gain;
    int i_gain;
    int d_gain;

};

class BaseJointState
{

public:
    BaseJointData curr_joint_state[ MAX_JOINT_ID + 1 ];
    BaseJointData goal_joint_state[ MAX_JOINT_ID + 1 ];
    BaseJointData fake_joint_state[ MAX_JOINT_ID + 1 ];

};

class BaseModule : public MotionModule
{
private:
    static BaseModule *unique_instance_;

    int                 control_cycle_msec_;
    boost::thread       queue_thread_;
    boost::thread*      tra_gene_tread_;

    ros::Publisher      status_msg_pub_;
    ros::Publisher      set_ctrl_module_pub_;

    std::map<std::string, int>  joint_name_to_id;

    BaseModule();

    void QueueThread();

    void parseIniPoseData( const std::string &path );
    void PublishStatusMsg(unsigned int type, std::string msg);

public:
    virtual ~BaseModule();

    static BaseModule *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );
    void    SetModeMsgCallback( const std_msgs::String::ConstPtr& msg );

    void    JointPoseMsgCallback( const manipulator_base_module_msgs::JointPose::ConstPtr& msg );
    void    KinematicsPoseMsgCallback( const manipulator_base_module_msgs::KinematicsPose::ConstPtr& msg );

    bool    GetJointPoseCallback( manipulator_base_module_msgs::GetJointPose::Request &req , manipulator_base_module_msgs::GetJointPose::Response &res );
    bool    GetKinematicsPoseCallback( manipulator_base_module_msgs::GetKinematicsPose::Request &req , manipulator_base_module_msgs::GetKinematicsPose::Response &res );

    /* ROS Calculation Functions */
    void    IniposeTraGeneProc( );
    void    JointTraGeneProc( );
    void    TaskTraGeneProc( );

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void    Stop();
    bool    IsRunning();

    /* Parameter */
    ManipulatorKinematicsDynamics *Manipulator;
    ROBOTIS_BASE::RobotisState *Robotis;
    BaseJointState *JointState;
};

}


#endif /* MANIPULATOR_BASEMODULE_H_ */
