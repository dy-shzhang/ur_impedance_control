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

//#include <boost/bind.hpp>

double RENEW_POSITION[6]={0.0,0.0,0.0,0.0,0.0,0.0};
double JOINT_VELOCITY[6]={0.0,0.0,0.0,0.0,0.0,0.0};
Eigen::Matrix<double,6,1> JOINT_SPEED_TARGET;
Eigen::Matrix<double,6,1> VELOCITY_TARGET;
//Eigen::Matrix<double,6,1> FORMER_VELOCITY_TARGET;
Eigen::MatrixXd JACOBIAN;
//Eigen::Vector3d RENEW_FORCE(0.0,0.0,0.0);//力传感器原始值
Eigen::Vector3d END_EFFORT_OFFSET(0.0,0.0823,0.0);//工作点相对wrist_3_Link的偏移量
Eigen::Vector3d CURRENT_POSITION(0,0,0);
Eigen::Matrix3d WRIST2FORCE;
Eigen::Matrix3d WORLD2FORCE;//原点到传感器的旋转矩阵

Eigen::Matrix<double,3,3> MF,MT;
Eigen::Matrix<double,3,3> BF,BT;
Eigen::Matrix<double,3,3> SF,ST;
Eigen::Matrix<double,3,1> TARGETFORCE;//笛卡尔空间中
Eigen::Matrix<double,3,1> TARGETTORQUE;
Eigen::Matrix<double,3,1> VEL_ERR_F;
Eigen::Matrix<double,3,1> ORI_ERR_F;

std::vector<Eigen::Vector3d>FORCE_FILTER;

std::vector<Eigen::Vector3d>TORQUE_FILTER;
// std::vector<Eigen::Vector3d>FORCE_TCP;
// std::vector<Eigen::Vector3d>TORQUE_TCP;

void getNewJointState(const sensor_msgs::JointState& msg){//更新关节位置
    for(int i=0;i<6;i++)
        RENEW_POSITION[i] = msg.position[i];
    
    for(int i=0;i<6;i++)
        JOINT_VELOCITY[i] =msg.velocity[i];
}

void getNewForce(const force_sensor::Force_Torque& msg){//更新传感器力矩值
    Eigen::Vector3d tmpForce(msg.fx,msg.fy,msg.fz);
    //RENEW_FORCE =tmpForce;
    FORCE_FILTER.push_back(tmpForce);

    Eigen::Vector3d tmpTorque(msg.tx,msg.ty,msg.tz);
    TORQUE_FILTER.push_back(tmpTorque);
}

// void getNewTcpFT(const geometry_msgs::Wrench& msg){ //实际上是算出来的,不可用波动太大
//     Eigen::Vector3d tmpForce(msg.force.x,msg.force.y,msg.force.z);
//     Eigen::Vector3d tmpTorque(msg.torque.x,msg.torque.y,msg.torque.z);
//     Eigen::Matrix3d changeToWorld;
//     changeToWorld<<-1.,0.,0.,
//                      0.,-1.,0,
//                      0,0,1;//wrench 使用的坐标系和我自己订的坐标系相差[0,0,180](我->速度);

//     FORCE_TCP.push_back(changeToWorld*tmpForce);
//     TORQUE_TCP.push_back(changeToWorld*tmpTorque);

// }

// void velocityFormer(){//对换向的速度进行平滑,看看能不能消除抖动
//     Eigen::Matrix<double,3,1>joint_speed_current;
//     joint_speed_current<<JOINT_VELOCITY[0],JOINT_VELOCITY[1],JOINT_VELOCITY[2];
//     Eigen::Matrix<double,3,1> current_speed = WORLD2FORCE.transpose()*joint_speed_current;
//     for(int i=0;i<3;i++){
//         if(current_speed(i)*VELOCITY_TARGET(i)<1e-7 && current_speed(i)>0.001)
//                     VELOCITY_TARGET(i)=0;
//     }

//     Eigen::Matrix<double,3,1>joint_orient_current;
//     joint_orient_current<<JOINT_VELOCITY[3],JOINT_VELOCITY[4],JOINT_VELOCITY[5];
//     Eigen::Matrix<double,3,1> current_orient = WORLD2FORCE.transpose()*joint_orient_current;
//     for(int i=3;i<6;i++){
//         if(current_orient(i-3)*VELOCITY_TARGET(i)<1e-7 && current_orient(i-3)>0.001)
//                     VELOCITY_TARGET(i)=0;
//     }
// }

// void forceAndTorqueDamping(){
    
// }

void deadZoo(){
    /*
    for(int i=0;i<6;i++){
        if((VELOCITY_TARGET(i)-JOINT_VELOCITY[i])<0.1 && (VELOCITY_TARGET(i)-JOINT_VELOCITY[i])>-0.1)
            VELOCITY_TARGET(i) =JOINT_VELOCITY[i];
    }*/

    for(int i=0;i<6;i++){
        if((VELOCITY_TARGET(i))<0.1 && (VELOCITY_TARGET(i))>-0.1)
             VELOCITY_TARGET(i) =0.;
    }

}
void initRobotState(){
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan planner;
    moveit::planning_interface::MoveItErrorCode flag;

    group.setGoalJointTolerance(0.01);
    group.setMaxVelocityScalingFactor(0.1);
    double tmp[6] ={-1.5708,-1.5708,-1.5708,-1.5708,1.5708,0.};
    std::vector<double> joint_value(tmp,tmp+6);
    group.setJointValueTarget(joint_value);
    flag = group.plan(planner);
    if(flag){
        group.execute(planner);
    }
    else{
        ROS_INFO("unable to init robot state");
    }

    while(ros::ok()){
        ros::spinOnce();
    }
    //to do 力传感器采集初始值,滤波

}

void impedanceControl(Eigen::Matrix<double,6,1>& fomer_force){
    //static Eigen::Vector3d fomer_force(0,0,0);
    Eigen::Vector3d filtered_force(0,0,0);
    Eigen::Vector3d filtered_torque(0,0,0);
    for(int i=0;i<5;i++){
        filtered_force += FORCE_FILTER[i];
        filtered_torque += TORQUE_FILTER[i];

    }
    if(FORCE_FILTER.size()==0){
        ROS_INFO("FORCE ERROR");
        for(int i=0;i<3;i++)
            filtered_force(i) = fomer_force(i,0);
        for(int i=0;i<3;i++)
         filtered_torque(i) = fomer_force(i+3,0);
    }
    else{
        filtered_force =filtered_force/FORCE_FILTER.size();
        filtered_torque =filtered_torque /TORQUE_FILTER.size();
    }
    for(int i=0;i<3;i++){
        fomer_force(i,0)=filtered_force(i);
    }
    for(int i=0;i<3;i++){
        fomer_force(i+3,0)=filtered_torque(i);
    }
    Eigen::Vector3d forceInWorld = WORLD2FORCE*filtered_force;
    Eigen::Vector3d torqueInWorld = WORLD2FORCE*filtered_torque;

    ROS_INFO_STREAM("FORCE_SENSOR \n"<<forceInWorld);
    ROS_INFO_STREAM("TORQUE_SENSOR \n"<<torqueInWorld);
    Eigen::Matrix3d velocityToWorld;
    velocityToWorld<<-1.,0.,0.,
                     0.,-1.,0,
                     0,0,1;//speedl 使用的坐标系和我自己订的坐标系相差[0,0,180](我->速度);
    Eigen::Vector3d err_f = forceInWorld - TARGETFORCE;//力和运动的方向是相同的
    for(int i=0;i<3;i++)
        if(err_f(i)<0.2 && err_f(i)>-0.2)
            err_f(i)=0.;
   // ROS_INFO_STREAM("err in force :"<<err_f);
    Eigen::Vector3d velocity = MF*(err_f - VEL_ERR_F)*0.00+BF*err_f*0.1;
    VEL_ERR_F=err_f;

    Eigen::Vector3d err_tor = torqueInWorld - TARGETTORQUE;
    for(int i=0;i<3;i++)
        if(err_tor(i)<0.05 && err_tor(i)>-0.05)
            err_tor(i)=0.;
   // ROS_INFO_STREAM("err in torque :"<<err_tor);
    Eigen::Vector3d orient = MT*(err_tor - ORI_ERR_F)*0.05+BT*err_tor*1;
    ORI_ERR_F =err_tor;

    Eigen::Matrix<double,6,1> velocities;
    velocity =velocityToWorld*velocity;
    orient = velocityToWorld * orient;
    velocities<<velocity(0),velocity(1),velocity(2),orient(0),orient(1),orient(2);
    
    JOINT_SPEED_TARGET = JACOBIAN.inverse()*velocities;//雅克比矩阵的逆*末端速度==关节速度
    
    VELOCITY_TARGET =velocities;
    
}

int main(int argc,char** argv){
    
    //初始化ros节点,robot_state
    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    
    ros::AsyncSpinner spinner(3);
    
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    std::vector<std::string>names = kinematic_state->getVariableNames();

    std::string frameNames = kinematic_model->getModelFrame();
    
    ros::NodeHandle node_handle;

    //计算wrist_3_link 到 force_sensor 的旋转矩阵
    Eigen::Matrix3d WRIST2TMP;
    WRIST2TMP<< 1,0,0,
                0,0,1,
                0,-1,0;
    //ROS_INFO_STREAM(WRIST2TMP);
    Eigen::Matrix3d TMP2FORCE;
    TMP2FORCE<<0.5,sqrt(3)/2.,0,
            -sqrt(3)/2.,  0.5,0,
                 0,    0,1;
    WRIST2FORCE = WRIST2TMP*TMP2FORCE;
    MF<<1,0,0,
        0,1,0,
        0,0,1;
    MT<<1.0,0,0,
        0,1.0,0,
        0,0,1.0;
    BF<<1,0,0,
        0,1,0,
        0,0,1;
    BT<<1.0,0,0,
        0,1.0,0,
        0,0,1.0;
    SF<<1.0,0,0,
        0,1.0,0,
        0,0,1.0;
    TARGETFORCE  << 0,0,0;
    TARGETTORQUE << 0,0,0;
    VEL_ERR_F<<0,0,0;
    ORI_ERR_F<<0,0,0;

    Eigen::Matrix<double,6,1> fomer_force;
    fomer_force<<0,0,0,0,0,0;

    std::string hello_world="hello world";

    ros::Subscriber subforce = node_handle.subscribe("/force", 1000, getNewForce);
    ros::Subscriber subjointstates = node_handle.subscribe("/joint_states", 1 ,&getNewJointState);

    ros::Publisher speed_pub =node_handle.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed",1);// /ur_driver/joint_speed
    ros::Publisher joint_record_pub =node_handle.advertise<ur_control::jointSpeedRecord>("/joint_speed_record",1);// /ur_driver/joint_speed

    ur_control::jointSpeedRecord speedRecord;
    speedRecord.angle.resize(6);
    trajectory_msgs::JointTrajectory msg;
    msg.points.resize(1);
    msg.points[0].velocities.resize(6);
    ros::Rate loop_rate(1250);
    int count =0;
    ros::Duration(0.5).sleep();//睡眠0.5s,让信息同步
    initRobotState();
    while(ros::ok()){
        //ros::spinOnce();
        if(count%5==4)
        {
            kinematic_state->setVariablePositions(RENEW_POSITION);//更新robot_state 的关节值

            kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("wrist_3_link"),END_EFFORT_OFFSET,JACOBIAN);//更新雅克比矩阵

            Eigen::Affine3d tf2 =kinematic_state->getGlobalLinkTransform("wrist_3_link");//实时获取world 到wrist_3_link 的 转移矩阵

            WORLD2FORCE =tf2.rotation()*WRIST2FORCE;
            
            impedanceControl(fomer_force);
            //velocityFormer();
            //deadZoo();

            for(int i=0;i<6;i++){
                //关节空间
           //     msg.points[0].velocities[i] =JOINT_SPEED_TARGET(i);
          //      speedRecord.angle[i] = JOINT_SPEED_TARGET(i);
                //笛卡尔空间,在ur_modern_driver 中的ur_ros_wrapper.cpp 里面改 speedInterface 函数
                msg.points[0].velocities[i] =VELOCITY_TARGET(i);
                speedRecord.angle[0] =VELOCITY_TARGET(i);
                
            }
            //z 方向限速
           // if(msg.points[0].velocities[2]>0.05)msg.points[0].velocities[2]=0.05;
           // else if(msg.points[0].velocities[2]<-0.05)msg.points[0].velocities[2]=-0.05;
           // speed_pub.publish(msg);
           // joint_record_pub.publish(speedRecord);
            count=-1;
            FORCE_FILTER.clear();

            TORQUE_FILTER.clear();

        }
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    ros::waitForShutdown();
    return 0;
}
