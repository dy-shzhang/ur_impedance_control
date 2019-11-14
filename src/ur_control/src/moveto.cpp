#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>
#include <ros/ros.h>
#include <iostream> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <force_sensor/Force_Torque.h>
#include <vector>
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h" 
#include <ur_control/jointSpeedRecord.h>

#define PI 3.141592653

//Eigen::Matrix<double,6,1> jointAngle;
//Eigen::Matrix<double,6,1> jointSpeed;

double angles[6];
double velocities[6];


void getNewJointState(const sensor_msgs::JointState& msg){
   // Eigen::Matrix<double,6,1> angle;
    for(int i=0;i<6;i++)
        angles[i] = msg.position[i];
    
    for(int i=0;i<6;i++)
        velocities[i] =msg.velocity[i];
}

int main(int argc,char** argv){
    ros::init(argc, argv, "try_vel", ros::init_options::AnonymousName);
    
    ros::AsyncSpinner spinner(1);
    
    spinner.start();
    ros::NodeHandle node_handle;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    ros::Subscriber subjointstates = node_handle.subscribe("/joint_states", 1 ,&getNewJointState);

    Eigen::Matrix3d r01,r12,r23,r34,r45,r56,r67;//c01,c12,c23,c34,c45,c56,c67;
    Eigen::Vector3d p01,p12,p23,p34,p45,p56,p67;//w0,w1,w2,w3,w4,w5,w6;

    r01<<-1,0,0,0,-1,0,0,0,1;
    //c01<<cos(angles[0]),-sin(angles[0]),0,sin(angles[0]),cos(angles[0]),0,0,0,1;//[0,0,1]
    p01<<0,0,0;

    r12<<0,0,1,0,1,0,-1,0,0;
    //c12<<cos(angles[1]),0,-sin(angles[1]),0,1,0,sin(angles[1]),0,cos(angles[1]); //[0,1,0]
    p12<<0,0.13585,0;
    
    r23<<1,0,0,0,1,0,0,0,1;
    //c23<<cos(angles[2]),0,-sin(angles[2]),0,1,0,sin(angles[2]),0,cos(angles[3]);//[0,1,0]
    p23<<0,-0.1197,0.70500;

    r34<<0,0,1,0,1,0,-1,0,0;
    //c34<<cos(angles[3]),0,-sin(angles[3]),0,1,0,sin(angles[3]),0,cos(angles[3]);//[0,1,0]
    p34<<0,0,0.64225;

    r45<<1,0,0,0,1,0,0,0,1;
    //c45<<cos(angles[4]),-sin(angles[4]),0,sin(angles[4]),cos(angles[4]),0,0,0,1; //[0,0,1]
    p45<<0,0.093,0;

    r56<<1,0,0,0,1,0,0,0,1;
    //c56<<cos(angles[5]),0,-sin(angles[5]),0,1,0,sin(angles[5]),0,cos(angles[5]); //[0,1,0]
    p56<<0,0,.09465;


    r67<<0,-1,0,1,0,0,0,0,1;
    //c67<<1,0,0,0,1,0,0,0,1;
    p67<<0,0.0823,0;
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        Eigen::Matrix3d c01,c12,c23,c34,c45,c56,c67;
        Eigen::Vector3d w0,w1,w2,w3,w4,w5;
        c01<<cos(angles[0]),-sin(angles[0]),0,sin(angles[0]),cos(angles[0]),0,0,0,1;//[0,0,1]
        c12<<cos(angles[1]),0,-sin(angles[1]),0,1,0,sin(angles[1]),0,cos(angles[1]); //[0,1,0]
        c23<<cos(angles[2]),0,-sin(angles[2]),0,1,0,sin(angles[2]),0,cos(angles[3]);//[0,1,0]
        c34<<cos(angles[3]),0,-sin(angles[3]),0,1,0,sin(angles[3]),0,cos(angles[3]);//[0,1,0]
        c45<<cos(angles[4]),-sin(angles[4]),0,sin(angles[4]),cos(angles[4]),0,0,0,1; //[0,0,1]
        c56<<cos(angles[5]),0,-sin(angles[5]),0,1,0,sin(angles[5]),0,cos(angles[5]); //[0,1,0]
        c67<<1,0,0,0,1,0,0,0,1;

        w0<<0,0,velocities[0];
        w1<<0,velocities[1],0;
        w2<<0,velocities[2],0;
        w3<<0,velocities[3],0;
        w4<<0,0,velocities[4];
        w5<<0,velocities[5],0;

        Eigen::Vector3d tcp;

        tcp =r01*(r12*(r23*(r34*(r45*(r56*p67+p56)+p45)+p34)+p23)+p12)+p01;
        ROS_INFO_STREAM(tcp);
        //(r01.transpose()*w0+w1)
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}