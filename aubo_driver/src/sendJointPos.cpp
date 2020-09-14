/*
Node runs at a designated frequency.
Gets a trajectory_msgs::JointTrajectoryPoint message (velocity part),
integrates it (velocity*(1/nodeFrequency)) and send to Aubo (using AUBO's
function sendPos2CanBus() ).
Feedback using last position sent.
*/

#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"

double jointVelocity[ARM_DOF];
double jointPositionDelta[ARM_DOF];
double jointPositionCommand[ARM_DOF];

void JointCommandCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{
    for (int i = 0; i < ARM_DOF; i++)
        jointVelocity[i] = msg->velocities.at(i);
}

void getJointAngle(aubo_driver::AuboDriver &robot_driver)
{
    aubo_robot_namespace::JointParam jointParam;

    int ret = robot_driver.robot_send_service_.robotServiceGetJointAngleInfo(jointParam);
	if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to get Joint Angle Info, error code:%d", ret);

	for (int i = 0; i < ARM_DOF; i++)
		jointPositionCommand[i] = jointParam.jointPos[i];
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "sendJointPos");

    ros::NodeHandle nh;

    ros::Subscriber jointVelocitySub = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>("/joint_command", 200, JointCommandCallback);

    aubo_driver::AuboDriver robot_driver;

	std::string serverHostParameter;
    ros::param::get("/aubo_driver/server_host", serverHostParameter);
	ros::param::set("/aubo_driver/server_host", (serverHostParameter=="")? "192.168.88.15" : serverHostParameter);

    robot_driver.run();

    ros::Duration(1).sleep(); // Time needed to initialize variables

    // IMPLEMENT: set node looprate through rosparam
    // double sendJointPosLoopRate;
    // if (!ros::param::get("/aubo_driver/sendJointPosLoopRate", sendJointPosLoopRate))
    //     ros::param::set("/aubo_driver/sendJointPosLoopRate", 100);

	//ros::param::get("/aubo_driver/sendJointPosLoopRate", sendJointPosLoopRate);
    // double frequency = sendJointPosLoopRate;
    // double samplingPeriod = 1/frequency;


    double frequency = 150.0;
    double samplingPeriod = 1/frequency;

    ros::Rate loop_rate(frequency);

    getJointAngle(robot_driver);

    while (ros::ok())
	{
        ros::spinOnce();

        for (int i = 0; i < ARM_DOF; i++)
        {
            jointPositionDelta[i] = jointVelocity[i]*samplingPeriod;
            jointPositionCommand[i] += jointPositionDelta[i];
        }

        loop_rate.sleep();

        int ret = robot_driver.robot_send_service_.robotServiceSetRobotPosData2Canbus(jointPositionCommand); 
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
           ROS_ERROR("Failed to move to position_sent, error code:%d", ret);
    }

    robot_driver.robot_send_service_.robotServiceLeaveTcp2CanbusMode();

    return 0;
}