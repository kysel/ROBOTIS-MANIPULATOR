/**
 * @file /include/robotis_manipulator_h_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotis_manipulator_gui_QNODE_HPP_
#define robotis_manipulator_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include <manipulator_base_module_msgs/JointPose.h>
#include <manipulator_base_module_msgs/KinematicsPose.h>

#include <manipulator_base_module_msgs/GetJointPose.h>
#include <manipulator_base_module_msgs/GetKinematicsPose.h>

#define tol_cnt     100

#define deg2rad 	(M_PI / 180.0)
#define rad2deg 	(180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_manipulator_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg, std::string sender="GUI");
    void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

    void sendIniPoseMsg( std_msgs::String msg );
    void sendSetModeMsg( std_msgs::String msg );

    void sendJointPoseMsg( manipulator_base_module_msgs::JointPose msg );
    void sendKinematicsPoseMsg( manipulator_base_module_msgs::KinematicsPose msg );

public Q_SLOTS:
    void getJointPose( std::vector<std::string> joint_name );
    void getKinematicsPose ( std::string group_name );

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    void update_curr_joint_pose( manipulator_base_module_msgs::JointPose );
    void update_curr_kinematics_pose( manipulator_base_module_msgs::KinematicsPose );

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Publisher ini_pose_msg_pub;
    ros::Publisher set_mode_msg_pub;

    ros::Publisher joint_pose_msg_pub;
    ros::Publisher kinematics_pose_msg_pub;

    ros::ServiceClient get_joint_pose_client;
    ros::ServiceClient get_kinematics_pose_client;

    ros::Subscriber status_msg_sub;

};

}  // namespace robotis_manipulator_gui

#endif /* robotis_manipulator_gui_QNODE_HPP_ */
