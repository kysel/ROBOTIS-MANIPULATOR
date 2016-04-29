 /*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "manipulator_base_module/BaseModule.h"

using namespace ROBOTIS;

BaseModule *BaseModule::unique_instance_ = new BaseModule();

BaseModule::BaseModule()
    : control_cycle_msec_(0)
{
    enable          = false;
    module_name     = "base_module";
    control_mode    = POSITION_CONTROL;

    result["joint1"] = new DynamixelState();
    result["joint2"] = new DynamixelState();
    result["joint3"] = new DynamixelState();
    result["joint4"] = new DynamixelState();
    result["joint5"] = new DynamixelState();
    result["joint6"] = new DynamixelState();

    joint_name_to_id["joint1"] = 1;
    joint_name_to_id["joint2"] = 2;
    joint_name_to_id["joint3"] = 3;
    joint_name_to_id["joint4"] = 4;
    joint_name_to_id["joint5"] = 5;
    joint_name_to_id["joint6"] = 6;

    Manipulator = new ManipulatorKinematicsDynamics( ARM );
    Robotis = new ROBOTIS_BASE::RobotisState();
    JointState = new BaseJointState();
}

BaseModule::~BaseModule()
{
    queue_thread_.join();
}

void BaseModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&BaseModule::QueueThread, this));

    ros::NodeHandle _ros_node;

    /* publish topics */
    status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    set_ctrl_module_pub_	= _ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
}

void BaseModule::parseIniPoseData( const std::string &path )
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    // parse movement time
    double _mov_time;
    _mov_time = doc["mov_time"].as< double >();

    Robotis->mov_time = _mov_time;

    // parse target pose
    YAML::Node _tar_pose_node = doc["tar_pose"];
    for(YAML::iterator _it = _tar_pose_node.begin() ; _it != _tar_pose_node.end() ; ++_it)
    {
        int _id;
        double _value;

        _id = _it->first.as<int>();
        _value = _it->second.as<double>();

        Robotis->joint_ini_pose.coeffRef( _id , 0 ) = _value * deg2rad;
    }

    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;
    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );
}

void BaseModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */
    ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/base/ini_pose_msg", 5, &BaseModule::IniPoseMsgCallback, this);
    ros::Subscriber set_mode_msg_sub = _ros_node.subscribe("/robotis/base/set_mode_msg", 5, &BaseModule::SetModeMsgCallback, this);

    ros::Subscriber joint_pose_msg_sub = _ros_node.subscribe("/robotis/base/joint_pose_msg", 5, &BaseModule::JointPoseMsgCallback, this);
    ros::Subscriber kinematics_pose_msg_sub = _ros_node.subscribe("/robotis/base/kinematics_pose_msg", 5, &BaseModule::KinematicsPoseMsgCallback, this);

    ros::ServiceServer get_joint_pose_server = _ros_node.advertiseService("/robotis/base/get_joint_pose", &BaseModule::GetJointPoseCallback, this);
    ros::ServiceServer get_kinematics_pose_server = _ros_node.advertiseService("/robotis/base/get_kinematics_pose", &BaseModule::GetKinematicsPoseCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
    }
}

void BaseModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    if( enable == false )
        return;

    if ( Robotis->is_moving == false )
    {
        if ( msg->data == "ini_pose")
        {
            // parse initial pose
            std::string _ini_pose_path = ros::package::getPath("manipulator_base_module") + "/config/ini_pose.yaml";
            parseIniPoseData( _ini_pose_path );

            tra_gene_tread_ = new boost::thread(boost::bind(&BaseModule::IniposeTraGeneProc, this));
            delete tra_gene_tread_;
        }
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void BaseModule::SetModeMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    std_msgs::String _msg;
    _msg.data = "base_module";

    set_ctrl_module_pub_.publish( _msg );

    return;
}

bool BaseModule::GetJointPoseCallback( manipulator_base_module_msgs::GetJointPose::Request &req , manipulator_base_module_msgs::GetJointPose::Response &res )
{
    if ( enable == false )
        return false;

    for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
    {
        for ( int _name_index = 0; _name_index < req.joint_name.size(); _name_index++ )
        {
            if( Manipulator->manipulator_link_data[ _id ]->name == req.joint_name[ _name_index ] )
            {
                res.joint_name.push_back( Manipulator->manipulator_link_data[ _id ]->name );
                res.joint_value.push_back( JointState->goal_joint_state[ _id ].position );

                break;
            }
        }
    }

    return true;
}

bool BaseModule::GetKinematicsPoseCallback( manipulator_base_module_msgs::GetKinematicsPose::Request &req , manipulator_base_module_msgs::GetKinematicsPose::Response &res )
{
    if ( enable == false )
        return false;

    res.group_pose.position.x = Manipulator->manipulator_link_data[ end_link ]->position.coeff(0,0);
    res.group_pose.position.y = Manipulator->manipulator_link_data[ end_link ]->position.coeff(1,0);
    res.group_pose.position.z = Manipulator->manipulator_link_data[ end_link ]->position.coeff(2,0);

    Eigen::Quaterniond _quaternion = rotation2quaternion( Manipulator->manipulator_link_data[ end_link ]->orientation );

    res.group_pose.orientation.w = _quaternion.w();
    res.group_pose.orientation.x = _quaternion.x();
    res.group_pose.orientation.y = _quaternion.y();
    res.group_pose.orientation.z = _quaternion.z();

    return  true;
}

void BaseModule::KinematicsPoseMsgCallback( const manipulator_base_module_msgs::KinematicsPose::ConstPtr& msg )
{
    if( enable == false )
        return;

    Robotis->kinematics_pose_msg = *msg;

    Robotis->ik_id_start = 0;
    Robotis->ik_id_end = end_link;

    if ( Robotis->is_moving == false )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&BaseModule::TaskTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void BaseModule::JointPoseMsgCallback( const manipulator_base_module_msgs::JointPose::ConstPtr& msg )
{
    if( enable == false )
        return;

    Robotis->joint_pose_msg = *msg;

    if ( Robotis->is_moving == false )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&BaseModule::JointTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void BaseModule::IniposeTraGeneProc()
{
    if( enable == false )
        return;

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_ini_pose.coeff( id , 0 );

        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                tar_value , 0.0 , 0.0 ,
                                                Robotis->smp_time , Robotis->mov_time );

        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->cnt = 0;
    Robotis->is_moving = true;

    ROS_INFO("[start] send trajectory");
    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void BaseModule::JointTraGeneProc()
{
    if( enable == false )
        return;

    /* set movement time */
    double _tol = 35 * deg2rad; // rad per sec
    double _mov_time = 2.0;

    double _diff , __diff ;
    _diff = 0.0;

    for ( int _name_index = 0; _name_index < Robotis->joint_pose_msg.name.size(); _name_index++ )
    {
        double _ini_value;
        double _tar_value;

        for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
        {
            if ( Manipulator->manipulator_link_data[ _id ]->name == Robotis->joint_pose_msg.name[ _name_index ] )
            {
                _ini_value = JointState->goal_joint_state[ _id ].position;
                _tar_value = Robotis->joint_pose_msg.value[ _name_index ];

                break;
            }
        }

        __diff = fabs ( _tar_value - _ini_value );

        if ( _diff < __diff )
            _diff = __diff;
    }

    Robotis->mov_time =  _diff / _tol;
    int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
    Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

    if ( Robotis->mov_time < _mov_time )
        Robotis->mov_time = _mov_time;

    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

    /* calculate joint trajectory */
    for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
    {
        double ini_value = JointState->goal_joint_state[ _id ].position;
        double tar_value;

        for ( int _name_index = 0; _name_index < Robotis->joint_pose_msg.name.size(); _name_index++ )
        {
            if ( Manipulator->manipulator_link_data[ _id ]->name == Robotis->joint_pose_msg.name[ _name_index ] )
            {
                tar_value = Robotis->joint_pose_msg.value[ _name_index ];
                break;
            }
        }

        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                tar_value , 0.0 , 0.0 ,
                                                              Robotis->smp_time , Robotis->mov_time );

        Robotis->calc_joint_tra.block( 0 , _id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->cnt = 0;
    Robotis->is_moving = true;

    ROS_INFO("[start] send trajectory");
    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void BaseModule::TaskTraGeneProc()
{
    /* set movement time */
    double _tol = 0.1; // m per sec
    double _mov_time = 2.0;

    double _diff = sqrt( pow( Manipulator->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( 0 , 0 ) - Robotis->kinematics_pose_msg.pose.position.x , 2  ) +
                         pow( Manipulator->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( 1 , 0 ) - Robotis->kinematics_pose_msg.pose.position.y , 2  ) +
                         pow( Manipulator->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( 2 , 0 ) - Robotis->kinematics_pose_msg.pose.position.z , 2  ) );

    Robotis->mov_time = _diff / _tol;
    int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
    Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

    if ( Robotis->mov_time < _mov_time )
        Robotis->mov_time = _mov_time;

    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_task_tra.resize( Robotis->all_time_steps , 3 );

     /* calculate trajectory */
    for ( int dim = 0; dim < 3; dim++ )
    {
        double ini_value = Manipulator->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( dim , 0 );
        double tar_value;

        if ( dim == 0 )
            tar_value = Robotis->kinematics_pose_msg.pose.position.x;
        else if ( dim == 1 )
            tar_value = Robotis->kinematics_pose_msg.pose.position.y;
        else if ( dim == 2 )
            tar_value = Robotis->kinematics_pose_msg.pose.position.z;

        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                tar_value , 0.0 , 0.0 ,
                                                Robotis->smp_time , Robotis->mov_time );

        Robotis->calc_task_tra.block( 0 , dim , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->cnt = 0;
    Robotis->is_moving = true;
    Robotis->ik_solve = true;

    ROS_INFO("[start] send trajectory");
    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void BaseModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable == false)
        return;

    /*----- write curr position -----*/

    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

        double _joint_curr_position = _dxl->dxl_state->present_position;
        double _joint_goal_position = _dxl->dxl_state->goal_position;

        JointState->curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
        JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;
    }

     /*----- forward kinematics -----*/

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        Manipulator->manipulator_link_data[ id ]->joint_angle = JointState->goal_joint_state[ id ].position;

    Manipulator->forwardKinematics( 0 );

    /* ----- send trajectory ----- */

//    ros::Time time = ros::Time::now();

    if ( Robotis->is_moving == true )
    {
        if ( Robotis->cnt == 0 )
            Robotis->ik_start_rotation = Manipulator->manipulator_link_data[ Robotis->ik_id_end ]->orientation;

        if ( Robotis->ik_solve == true )
        {
            Robotis->setInverseKinematics( Robotis->cnt , Robotis->ik_start_rotation );

            int max_iter = 30;
            double ik_tol = 1e-3;
            bool ik_success = Manipulator->inverseKinematics( Robotis->ik_id_start , Robotis->ik_id_end ,
                                                        Robotis->ik_target_position , Robotis->ik_target_rotation ,
                                                        max_iter , ik_tol );

            if ( ik_success == true )
            {
                for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                    JointState->goal_joint_state[ id ].position = Manipulator->manipulator_link_data[ id ]->joint_angle;
            }
            else
            {
                ROS_INFO("[end] send trajectory (ik failed)");
                PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory (IK Failed)");

                Robotis->is_moving = false;
                Robotis->ik_solve = false;
                Robotis->cnt = 0;
            }
        }
        else
        {
            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                JointState->goal_joint_state[ id ].position = Robotis->calc_joint_tra( Robotis->cnt , id );
        }

        Robotis->cnt++;
    }

    /*----- set joint data -----*/

    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;
        result[_joint_name]->goal_position = JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position;
    }

    /*---------- initialize count number ----------*/     

    if ( Robotis->cnt >= Robotis->all_time_steps && Robotis->is_moving == true )
    {
        ROS_INFO("[end] send trajectory");
        PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

        Robotis->is_moving = false;
        Robotis->ik_solve = false;
        Robotis->cnt = 0;
    }
}

void BaseModule::Stop()
{
    Robotis->is_moving = false;
    Robotis->ik_solve = false;
    Robotis->cnt = 0;

    return;
}

bool BaseModule::IsRunning()
{
    return Robotis->is_moving;
}

void BaseModule::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Base";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}
