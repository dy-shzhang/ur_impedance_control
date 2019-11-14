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

class ImpedanceControl{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_force;
    ros::Subscriber sub_jointState;
    ros::Publisher pub_speed;
    ros::Publisher pub_record;
    ros::Rate loop_rate;
    double PI;
    //机器人模型相关
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_state::RobotStatePtr kinematic_state;
    robot_model::RobotModelPtr kinematic_model;
    const robot_state::JointModelGroup* joint_model_group;

    moveit::planning_interface::MoveGroupInterface group;
    moveit::planning_interface::MoveGroupInterface::Plan planner;
    moveit::planning_interface::MoveItErrorCode planSuccess;

    //msg 相关
    ur_control::jointSpeedRecord currentSpeedRecordMsg; //float64[6]
    trajectory_msgs::JointTrajectory cartesianVelocityTargetMsg;
    
    //关节空间相关变量
    double currentJointPosition[6]; //当前关节位置
    double currentJointVelocity[6]; //当前关节速度
    double toolVelocityRecord[6]; //速度记录
    double cartesianForceRecord[6];
    Eigen::Matrix<double,6,1> jointSpeedTarget;
    Eigen::Matrix<double,6,1> jointVelocityTarget;

    //cartesian 空间相关变量
    Eigen::Vector3d rawCartesianForce; //原始值-force
    Eigen::Vector3d rawCartesianTorque; //原始值-torque
    Eigen::Vector3d currentCartesianForce; //当前末端笛卡尔力,世界坐标系下
    Eigen::Vector3d currentCartesianTorque;//当前末端笛卡尔力矩
    Eigen::Matrix<double,3,1> cartesianForceTarget;//笛卡尔空间中力期望值,世界坐标系下
    Eigen::Matrix<double,3,1> cartesianTorqueTarget; //期望力矩值,世界坐标系下
    Eigen::Matrix<double,3,1> cartesianXYZTarget; //笛卡尔xyz速度信息
    Eigen::Matrix<double,3,1> cartesianRPYTarget; //笛卡尔rpy速度信息



    //坐标系转换
    Eigen::Matrix3d wrist2force;//wrist_3 到 力传感器坐标系的旋转举矩阵
    Eigen::Matrix3d world2force;//原点到传感器的旋转矩阵
    Eigen::MatrixXd jacobian;

    //其他变量
    Eigen::Vector3d endEffectOffset;//工作点相对wrist_3_Link的偏移量
    Eigen::Matrix<double,3,3> MF,MT;
    Eigen::Matrix<double,3,3> BF,BT;
    Eigen::Matrix<double,3,3> KF,KT;
    double detaT;

//临时变量,不对外开放
private:
    Eigen::Matrix<double,3,3>velocity2world; //转换速度坐标系和全局坐标系
    Eigen::Vector3d errInForce;
    Eigen::Vector3d perErrInForce;
    Eigen::Vector3d errInTorque;
    Eigen::Vector3d perErrInTorque;
    Eigen::Affine3d tf; //world to wrist_3_link

    Eigen::Vector3d forceBias;//力漂移
    Eigen::Vector3d torqueBias;//力矩漂移


public:
     ImpedanceControl():robot_model_loader("robot_description"),
                        kinematic_state(new robot_state::RobotState(robot_model_loader.getModel())),
                        group("manipulator"),
                        loop_rate(125){
        kinematic_model = robot_model_loader.getModel();
        joint_model_group = kinematic_model->getJointModelGroup("manipulator");

        sub_force = nh_.subscribe("/force", 1, &ImpedanceControl::getNewForceTorque,this);
        sub_jointState =nh_.subscribe("/joint_states", 1 ,&ImpedanceControl::getNewJointState,this);
        pub_speed =nh_.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed",1);// /ur_driver/joint_speed
        pub_record=nh_.advertise<ur_control::jointSpeedRecord>("/joint_speed_record",1);// /joint_speed_record

        cartesianVelocityTargetMsg.points.resize(1);
        cartesianVelocityTargetMsg.points[0].velocities.resize(6);
        cartesianVelocityTargetMsg.points[0].accelerations.resize(1);
        currentSpeedRecordMsg.angle.resize(6);

        cartesianVelocityTargetMsg.points[0].accelerations[0]=3;

        PI=3.141592653589793238;
        endEffectOffset<<0.0,0.0823,0.0;
        Eigen::Matrix3d WRIST2TMP,TMP2FORCE;
        WRIST2TMP<< 1,0,0, 0,0,1, 0,-1,0;
        TMP2FORCE<<0.5,sqrt(3)/2.,0, -sqrt(3)/2.,0.5,0, 0,0,1;
        //TMP2FORCE<<-sqrt(3)/2.,0.5,0,-0.5,-sqrt(3)/2.,0,0,0,1;
        wrist2force = WRIST2TMP*TMP2FORCE;
        MF<<1.,0.,0., 0.,1.,0., 0.,0.,1.;
        MT<<1,0.,0., 0.,1,0., 0.,0.,1;
        BF<<1,0.,0., 0.,1,0., 0.,0.,1;
        BT<<1.,0.,0., 0.,1.,0., 0.,0.,1.;
        KF<<1.,0.,0., 0.,1.,0., 0.,0.,1.;
        currentCartesianForce << 0.,0.,0.;
        currentCartesianTorque << 0.,0.,0.;
        cartesianForceTarget << 0,0,0;
        cartesianTorqueTarget << 0,0,0;
        perErrInForce<<0.,0.,0.;
        perErrInTorque<<0.,0.,0.;
        velocity2world<<-1.,0.,0., 0.,-1.,0., 0.,0.,1.;//speedl 使用的坐标系和我自己订的坐标系相差[0,0,180](我->速度);
        forceBias << 0,0,0;
        torqueBias << 0,0,0;
        detaT =0.008;
        for(int i=0;i<6;i++){
            toolVelocityRecord[i]=0;
            cartesianForceRecord[i]=0;
        }
    
    }
    bool initRobotState(){ //初始化robot 到基础位置
        group.setGoalJointTolerance(0.001);
        group.setMaxVelocityScalingFactor(0.1);
        double angle[6]={-PI/2.,-PI/2.,-PI/2.,-PI/2.,PI/2.,0.};
        std::vector<double> joint_value(angle,angle+6);
        group.setJointValueTarget(joint_value);
        planSuccess = group.plan(planner);
        if(planSuccess){
            group.execute(planner);
            return true;
        }
        else{
            ROS_INFO("unable to init robot state");
            return false;
        }
    }

    void getNewJointState(const sensor_msgs::JointState& msg){//更新关节位置,关节速度
        for(int i=0;i<6;i++)
            currentJointPosition[i] = msg.position[i];
        for(int i=0;i<6;i++)
            currentJointVelocity[i] =msg.velocity[i];
    }

    void getNewForceTorque(const force_sensor::Force_Torque& msg){//更新笛卡尔传感器力矩值
        Eigen::Vector3d tmpForce(msg.fx,msg.fy,msg.fz);
        rawCartesianForce = world2force*tmpForce;

        // for(int i=0;i<3;i++){
        //     if(rawCartesianForce(i)>5)
        //         rawCartesianForce(i) = 5;
        //     else if(rawCartesianForce(i) <-5)
        //         rawCartesianForce(i) =-5;
        // }
        //ROS_INFO_STREAM("force"<<rawCartesianForce);

        Eigen::Vector3d tmpTorque(msg.tx,msg.ty,msg.tz);
        rawCartesianTorque = world2force*tmpTorque;
        //ROS_INFO_STREAM("torque"<<rawCartesianTorque);
    }

    void deadZoom(Eigen::Vector3d& matrx,double range =0.5){
        for(int i=0;i<3;i++){
            if(matrx(i,0)<range && matrx(i,0)>range*-1)
                matrx(i,0) = 0.;
        }
    }
    void meanValueForceTorqueFilter(double times=5.){

        // static int count =0;
        // ROS_INFO_STREAM("COUNT ="<<count);
        // if(count==0){
        //     currentCartesianForce = currentCartesianForce-currentCartesianForce;
        //     currentCartesianTorque =currentCartesianTorque -currentCartesianTorque;
        // }
        // currentCartesianForce =currentCartesianForce + rawCartesianForce/times;
        // currentCartesianTorque = currentCartesianTorque +rawCartesianTorque/times;
        // if(count==4)
        //     count=0;
        // else
        //     count++;

        currentCartesianForce = rawCartesianForce - forceBias;
        currentCartesianTorque = rawCartesianTorque - torqueBias;

    }
    void impedanceControlPID(){

        Eigen::Matrix<double,6,1> endVelocity = getEndVelocity();
        Eigen::Matrix<double,3,1> XYZVelocity,RPYVelocity;
        XYZVelocity << endVelocity(0,0),endVelocity(1,0),endVelocity(2,0);
        RPYVelocity << endVelocity(3,0),endVelocity(4,0),endVelocity(5,0);
        // XYZVelocity = velocity2world * XYZVelocity;
        // RPYVelocity = velocity2world * RPYVelocity;
        // 速度方向
        for(int i=0;i<3;i++){
            currentSpeedRecordMsg.angle[i+3] = -1*XYZVelocity(i);
        }

        Eigen::Vector3d errInForce;
        Eigen::Vector3d errInTorque;

        errInForce= currentCartesianForce - cartesianForceTarget;//力和运动的方向是相同的
        //deadZoom(errInForce,0.1);
        cartesianXYZTarget =MF*(errInForce-perErrInForce)*0.005+BF*errInForce*0.001;
        cartesianXYZTarget =velocity2world*cartesianXYZTarget;
        perErrInForce =errInForce;  

        errInTorque = currentCartesianTorque - cartesianTorqueTarget;
       // deadZoom(errInTorque,0.002);
        cartesianRPYTarget = MT*(errInTorque - perErrInTorque)*5+BT*errInTorque*5;
        cartesianRPYTarget =velocity2world*cartesianRPYTarget;
        perErrInTorque =errInTorque;
    }

    void impedanceControl(){
        Eigen::Matrix<double,6,1> endVelocity = getEndVelocity();
        Eigen::Matrix<double,3,1> XYZVelocity,RPYVelocity;
        XYZVelocity << endVelocity(0,0),endVelocity(1,0),endVelocity(2,0);
        RPYVelocity << endVelocity(3,0),endVelocity(4,0),endVelocity(5,0);
        // XYZVelocity = velocity2world * XYZVelocity;
        // RPYVelocity = velocity2world * RPYVelocity;
        // 速度方向
        for(int i=0;i<3;i++){
            currentSpeedRecordMsg.angle[i+3] = -1*XYZVelocity(i);
        }
        //ROS_INFO_STREAM("current velocity "<<XYZVelocity);
        cartesianXYZTarget = (1*MF+BF).inverse()*(currentCartesianForce- cartesianForceTarget +1*MF*XYZVelocity);
        cartesianRPYTarget =(1*MT+BT).inverse()*(currentCartesianTorque - cartesianTorqueTarget +1*MT*RPYVelocity);

        cartesianXYZTarget =velocity2world*cartesianXYZTarget;
        cartesianRPYTarget =velocity2world*cartesianRPYTarget;


        // for(int i=0;i<3;i++){
        //     if(cartesianXYZTarget(i)<0.05 && cartesianXYZTarget(i)>=0)
        //         cartesianXYZTarget(i) =20*cartesianXYZTarget(i)*cartesianXYZTarget(i);
        //     else if(cartesianXYZTarget(i)>-0.05 && cartesianXYZTarget(i)<0)
        //         cartesianXYZTarget(i) =-20*cartesianXYZTarget(i)*cartesianXYZTarget(i);
        //     // if(cartesianXYZTarget(i)<0.001 && cartesianXYZTarget(i)>-0.001)
        //     //     cartesianXYZTarget(i)=0.;

        //     // if(cartesianRPYTarget(i)<0.05 && cartesianRPYTarget(i)>=0)
        //     //     cartesianRPYTarget(i) =20*cartesianRPYTarget(i)*cartesianRPYTarget(i);
        //     // else if(cartesianRPYTarget(i)>-0.05 && cartesianRPYTarget(i)<0)
        //     //     cartesianRPYTarget(i) =-20*cartesianRPYTarget(i)*cartesianRPYTarget(i);
        //     // if(cartesianRPYTarget(i)<0.001 && cartesianRPYTarget(i)>-0.001)
        //     //     cartesianRPYTarget(i)=0.;
        // }
    }

    Eigen::Matrix<double,6,1> getEndVelocity(){
        Eigen::Matrix<double,6,1> endEffortVelocity,tmpJointVelocity;
        tmpJointVelocity<<currentJointVelocity[0],currentJointVelocity[1],currentJointVelocity[2],currentJointVelocity[3],currentJointVelocity[4],currentJointVelocity[5];
        endEffortVelocity = jacobian * tmpJointVelocity;
        //ROS_INFO_STREAM(endEffortVelocity);
        return endEffortVelocity;

    }
    void run(){
        bool initDone =initRobotState(); //默认位置
        if(!initDone){
            ROS_INFO("robot init err, system shut down");
            return;
        }
        ROS_INFO("robot inited");
        int pause;
        ROS_INFO("push any key to continue :");
        std::cin>>pause;
        int countTime=0;
        bool sensorBias =true; //当为true时候,采集bias的值;为false时候进入控制模式
    
        while(ros::ok()){
            ros::spinOnce();
            kinematic_state->setVariablePositions(currentJointPosition);
            kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("wrist_3_link"),endEffectOffset,jacobian);
            tf =kinematic_state->getGlobalLinkTransform("wrist_3_link");
            world2force = tf.rotation()*wrist2force;
            getEndVelocity();
            if(sensorBias && countTime<10){
                if(rawCartesianForce(0)<0.0001 && rawCartesianForce(0)>-0.0001){
                }
                else{
                    for(int i=0;i<3;i++){
                        forceBias(i) += rawCartesianForce(i)/10.;
                        torqueBias(i) += rawCartesianTorque(i)/10.;
                    }
                    countTime++;
                    if(countTime==10)
                        sensorBias=false;
                }
            }
            else{
                meanValueForceTorqueFilter();
                impedanceControlPID();
                //impedanceControl();
            }
            //ROS_INFO_STREAM("target velocity "<<cartesianXYZTarget);

            ROS_INFO_STREAM(currentCartesianForce);
            cartesianVelocityTargetMsg.points[0].accelerations[0]=3;
            for(int i=0;i<3;i++){
                cartesianVelocityTargetMsg.points[0].velocities[i] = cartesianXYZTarget(i);//cartesianXYZTarget(i)
                cartesianVelocityTargetMsg.points[0].velocities[i+3] = 0;//cartesianRPYTarget(i)

                currentSpeedRecordMsg.angle[i] = cartesianXYZTarget(i);
                //currentSpeedRecordMsg.angle[i+3] = currentCartesianTorque(i);

                toolVelocityRecord[i] = cartesianXYZTarget(i);
                toolVelocityRecord[i+3] = cartesianRPYTarget(i);

                cartesianForceRecord[i] = currentCartesianForce(i);
                cartesianForceRecord[i+3] = currentCartesianTorque(i);
            }
            
            pub_speed.publish(cartesianVelocityTargetMsg);
            pub_record.publish(currentSpeedRecordMsg);
            loop_rate.sleep();
        }
        ros::waitForShutdown();
    }

};

int main(int argc,char** argv){

    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ImpedanceControl imp_;
    imp_.run();
    return 0;
}