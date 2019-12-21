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
    Eigen::Vector3d rawForce,rawTorque;

    Eigen::Vector3d currentCartesianForce; //当前末端笛卡尔力,世界坐标系下
    Eigen::Vector3d currentCartesianTorque;//当前末端笛卡尔力矩
    Eigen::Matrix<double,3,1> cartesianForceTarget;//笛卡尔空间中力期望值,世界坐标系下
    Eigen::Matrix<double,3,1> cartesianTorqueTarget; //期望力矩值,世界坐标系下
    Eigen::Matrix<double,3,1> cartesianXYZTarget; //笛卡尔xyz速度信息
    Eigen::Matrix<double,3,1> cartesianXYZFormer;
    Eigen::Matrix<double,3,1> cartesianRPYTarget; //笛卡尔rpy速度信息
    Eigen::Matrix<double,3,1> cartesianRPYFormer;



    //坐标系转换
    Eigen::Matrix3d wrist2force;//wrist_3 到 力传感器坐标系的旋转举矩阵
    Eigen::Matrix3d world2force;//原点到传感器的旋转矩阵
    Eigen::MatrixXd jacobian;

    //其他变量
    Eigen::Vector3d endEffectOffset;//工作点相对wrist_3_Link的偏移量
    Eigen::Matrix<double,3,3> MF,BF;
    Eigen::Matrix<double,3,3> MT,BT;
    Eigen::Matrix<double,3,3> PF,DF;
    Eigen::Matrix<double,3,3> PT,DT;
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

    Eigen::Vector3d MaxForce; //力/力矩上下限
    Eigen::Vector3d MaxTorque;

    double start_position[6];
    enum {APPROACHING=1,ROTATING,INSERTING,PULLBACK,STOP} STATE;
    double platform; //记录平台高度,用来配合判断插孔阶段


public:
     ImpedanceControl():robot_model_loader("robot_description"),
                        kinematic_state(new robot_state::RobotState(robot_model_loader.getModel())),
                        group("manipulator"),
                        loop_rate(125){
        kinematic_model = robot_model_loader.getModel();
        joint_model_group = kinematic_model->getJointModelGroup("manipulator");

        sub_force = nh_.subscribe("/force", 1, &ImpedanceControl::getNewForceTorque,this);
        sub_jointState =nh_.subscribe("/joint_states", 1 ,&ImpedanceControl::getNewJointState,this);
        pub_speed =nh_.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed",10);// /ur_driver/joint_speed
        pub_record=nh_.advertise<ur_control::jointSpeedRecord>("/joint_speed_record",10);// /joint_speed_record

        cartesianVelocityTargetMsg.points.resize(1);
        cartesianVelocityTargetMsg.points[0].velocities.resize(6);
        cartesianVelocityTargetMsg.points[0].accelerations.resize(1);
        currentSpeedRecordMsg.angle.resize(6);

        cartesianVelocityTargetMsg.points[0].accelerations[0]=3;

        PI=3.141592653589793238;
        endEffectOffset<<0.0,0.3232,0.0; //+ 0,0.25,0 是手爪末端位置
        Eigen::Matrix3d WRIST2TMP,TMP2FORCE;
        WRIST2TMP<< 1,0,0, 0,0,1, 0,-1,0;
        TMP2FORCE<<0.5,sqrt(3)/2.,0, -sqrt(3)/2.,0.5,0, 0,0,1;

        wrist2force = WRIST2TMP*TMP2FORCE;

        MF<< 0.,0.,0., 0.,0.,0., 0.,0.,0.; 
        BF<< 0.,0.,0., 0.,0.,0., 0.,0.,0.;  
        MT<< 0.,0.,0., 0.,0.,0., 0.,0.,0.;
        BT<< 0.,0.,0., 0.,0.,0., 0.,0.,0;

        PF<< 0.,0.,0., 0.,0.,0., 0.,0.,0.; 
        PT<< 0.,0.,0., 0.,0.,0., 0.,0.,0.;

        DF<< 0.,0.,0., 0.,0.,0., 0.,0.,0.;  
        DT<< 0.,0.,0., 0.,0.,0., 0.,0.,0;

        cartesianForceTarget << 0.,0.,0.;
        cartesianTorqueTarget <<0.0,0.,0.;


        currentCartesianForce << 0.,0.,0.;
        currentCartesianTorque << 0.,0.,0.;

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

        cartesianXYZFormer << 0,0,0;
        cartesianRPYFormer << 0,0,0;
        platform =0.192;
        STATE = APPROACHING;
        MaxForce<<50,50,50;
        MaxTorque<<0.5,0.5,0.5;
    
    }

    void init(){
        getParam();
    }

    bool initRobotState(){ //初始化robot 到基础位置
        group.setGoalJointTolerance(0.001);
        group.setMaxVelocityScalingFactor(0.1);
        //double angle[6]={-PI/2.,-PI/2.+0.18,-PI/2.+0.18,-PI/2.,PI/2.,0.};
        //double angle[6]={-90.,-85.,-80.,-103.,90.,3.};
        std::vector<double> joint_value(start_position,start_position+6);
        for(int i=0;i<6;i++){
            joint_value[i] = joint_value[i]/180.*PI;
        }
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
        
        renewKinState();
    }

    void getNewForceTorque(const force_sensor::Force_Torque& msg){//更新笛卡尔传感器力矩值

        Eigen::Vector3d tmpForce(msg.fx,msg.fy,msg.fz);
        rawForce = tmpForce;
        rawCartesianForce = world2force*tmpForce;

        Eigen::Vector3d tmpTorque(msg.tx,msg.ty,msg.tz);
        rawTorque = tmpTorque;
        rawCartesianTorque = world2force*tmpTorque;
    }

    void deadZoom(Eigen::Vector3d& matrx,double range =0.5){
        for(int i=0;i<3;i++){
            if(matrx(i,0)<range && matrx(i,0)>range*-1)
                matrx(i,0) = 0.;
        }
    }
    void meanValueForceTorqueFilter(double times=5.){

        currentCartesianForce = rawCartesianForce - forceBias;
        currentCartesianTorque = rawCartesianTorque - torqueBias;

        rawForce  = rawForce - world2force.inverse()*forceBias;
        rawTorque = rawTorque - world2force.inverse()*torqueBias;

    }
    void impedanceControlPID(){

        Eigen::Vector3d errInForce;
        Eigen::Vector3d errInTorque;

        errInForce= currentCartesianForce - cartesianForceTarget;//力和运动的方向是相同的
        cartesianXYZTarget =MF*(errInForce-perErrInForce)*0.005+BF*errInForce*0.001;
        cartesianXYZTarget =velocity2world*cartesianXYZTarget;
        perErrInForce =errInForce;  

        errInTorque = currentCartesianTorque - cartesianTorqueTarget;
        cartesianRPYTarget = MT*(errInTorque - perErrInTorque)*5+BT*errInTorque*5;
        cartesianRPYTarget =velocity2world*cartesianRPYTarget;
        perErrInTorque =errInTorque;

        if(cartesianXYZTarget(2)>0.1 || cartesianXYZTarget(2)<-0.1){
            if(cartesianXYZTarget(2)>0.1)
                cartesianXYZTarget(2) = 0.1;
            else
                cartesianXYZTarget(2) = -0.1;
        }

    }

    void impedanceControl(){
        Eigen::Matrix<double,6,1> endVelocity = getEndVelocity();
        Eigen::Matrix<double,3,1> XYZVelocity,RPYVelocity;
        XYZVelocity << endVelocity(0,0),endVelocity(1,0),endVelocity(2,0);
        RPYVelocity << endVelocity(3,0),endVelocity(4,0),endVelocity(5,0);

        cartesianXYZTarget = (125*MF+BF).inverse()*(currentCartesianForce- cartesianForceTarget + 125*MF*cartesianXYZFormer); //currentCartesianForce- cartesianForceTarget
        cartesianXYZFormer = cartesianXYZTarget;
        for(int i=0;i<3;i++){
                if(cartesianXYZTarget(i)<0.0005 && cartesianXYZTarget(i)>-0.0005)
                    cartesianXYZTarget(i) = 0.;
        }
        cartesianXYZFormer = cartesianXYZTarget;

        cartesianRPYTarget =(125*MT+BT).inverse()*(currentCartesianTorque - cartesianTorqueTarget +125*MT*cartesianRPYFormer);

        cartesianRPYFormer =cartesianRPYTarget;

        cartesianXYZTarget =velocity2world*cartesianXYZTarget;

        cartesianRPYTarget =velocity2world*cartesianRPYTarget;

        if(cartesianXYZTarget(2)>0.1 || cartesianXYZTarget(2)<-0.1){
            if(cartesianXYZTarget(2)>0.1)
                cartesianXYZTarget(2) = 0.1;
            else
                cartesianXYZTarget(2) = -0.1;
        }
    }

    void impedanceControlInSensorFrame(){ //在传感器坐标系下的阻抗控制
        cartesianXYZTarget = (125*MF+BF).inverse()*(rawForce- cartesianForceTarget + 125*MF*cartesianXYZFormer);

        cartesianXYZFormer = cartesianXYZTarget;

        cartesianRPYTarget =(125*MT+BT).inverse()*(rawTorque - cartesianTorqueTarget +125*MT*cartesianRPYFormer);

        cartesianRPYFormer =cartesianRPYTarget;

        cartesianXYZTarget =velocity2world*world2force*cartesianXYZTarget;

        cartesianRPYTarget =velocity2world*world2force*cartesianRPYTarget;
    }

    Eigen::Matrix<double,6,1> getEndVelocity(){
        Eigen::Matrix<double,6,1> endEffortVelocity,tmpJointVelocity;
        tmpJointVelocity<<currentJointVelocity[0],currentJointVelocity[1],currentJointVelocity[2],currentJointVelocity[3],currentJointVelocity[4],currentJointVelocity[5];
        endEffortVelocity = jacobian * tmpJointVelocity;
        //ROS_INFO_STREAM(endEffortVelocity);
        return endEffortVelocity;

    }

    void PTHybirControl(){
        bool initDone =initRobotState(); //默认位置
        if(!initDone){
            ROS_INFO("robot init err, system shut down");
            return;
        }
        ROS_INFO("robot inited");
        int pause;
        ROS_INFO("push any key to continue :");
        std::cin>>pause;

        double x=110,y=486,z=430;  //毫米.圆心?
        double xtarget =200;

        int countTime=0;
        while(ros::ok()){
            ros::spinOnce();
            kinematic_state->setVariablePositions(currentJointPosition);
            kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("wrist_3_link"),endEffectOffset,jacobian);
            tf =kinematic_state->getGlobalLinkTransform("wrist_3_link");
            Eigen::Vector3d currentCartesianEndEffortPositon = tf.rotation()*endEffectOffset+tf.translation();
            xtarget = 125*(0.1*sin(0.008*countTime)+0.11-currentCartesianEndEffortPositon(0));
            cartesianVelocityTargetMsg.points[0].accelerations[0]=1;
            for(int i=0;i<3;i++){
                cartesianVelocityTargetMsg.points[0].velocities[i] = 0;
                cartesianVelocityTargetMsg.points[0].velocities[i+3] = 0;
            }
            currentSpeedRecordMsg.angle[0] = 0.1*sin(0.008*countTime)+0.11;
            currentSpeedRecordMsg.angle[1] = currentCartesianEndEffortPositon(0);
            cartesianVelocityTargetMsg.points[0].velocities[0] = xtarget;
            pub_speed.publish(cartesianVelocityTargetMsg);
            pub_record.publish(currentSpeedRecordMsg);
            countTime+=1;
            loop_rate.sleep();
        }
        ros::waitForShutdown();

    }


    void renewKinState(){
        kinematic_state->setVariablePositions(currentJointPosition);
        kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("wrist_3_link"),endEffectOffset,jacobian);
        tf =kinematic_state->getGlobalLinkTransform("wrist_3_link");
        world2force = tf.rotation()*wrist2force;
    }


    Eigen::Vector3d getEndPosition(){
        Eigen::Vector3d currentCartesianEndEffortPositon = tf.rotation()*endEffectOffset+tf.translation();
        return currentCartesianEndEffortPositon;
    }
    
    void checkState(){ //判断插孔状态
        Eigen::Vector3d currentCartesianEndEffortPositon = getEndPosition();

        // Eigen::Vector3d tmpforce = currentCartesianForce -MaxForce;
        // Eigen::Vector3d tmptorque = currentCartesianTorque - MaxTorque;
        // for(int i=0;i<3 && STATE!=STOP;i++){
        //     if(tmpforce(i)>0 || tmptorque(i)>0)
        //         STATE =STOP;
        // }

        // tmpforce = -MaxForce - currentCartesianForce;
        // tmptorque = - MaxTorque - currentCartesianTorque ;
        // for(int i=0;i<3 && STATE!=STOP;i++){
        //     if(tmpforce(i)>0 || tmptorque(i)>0)
        //         STATE = STOP;
        // }
        ROS_INFO_STREAM("end link position "<<currentCartesianEndEffortPositon);
        switch(STATE){
            case APPROACHING:{
                if(currentCartesianForce(2)>cartesianForceTarget(2)){
                    platform = currentCartesianEndEffortPositon(2);
                    ROS_INFO_STREAM("end link record "<<platform);
                    STATE = ROTATING;
                }
            }break;
            case ROTATING:{
                if(platform-currentCartesianEndEffortPositon(2)>0.001 && currentCartesianForce(2)< cartesianForceTarget(2)){
                    STATE = INSERTING;
                }
            }break;
            case INSERTING:{
                if(platform-currentCartesianEndEffortPositon(2)>0.03){
                    STATE=STOP;
                }
            }break;
            case STOP:break;
            default:ROS_INFO_STREAM("ERROR");break;
        }
    }

    void pull_back(){  //回退到初始位置
        while(ros::ok()){
            Eigen::Vector3d endposition = getEndPosition();
            ROS_INFO_STREAM(endposition);
            if(endposition(2) >platform){
                ROS_INFO("BREAKUP");
                break;
            }
            ROS_INFO("PID PULLBACK");
            impedanceControlPID();
            cartesianXYZTarget(2)=0.01;
            bool flag =publicControlMessage();
            loop_rate.sleep();
        }
        bool initDone =initRobotState(); //默认位置
        if(!initDone){
            ROS_INFO("robot init err, system shut down");
            return;
        }
        ROS_INFO("pull back successful");
    }

    bool publicControlMessage(){
        cartesianVelocityTargetMsg.points[0].accelerations[0]=3 ;
        for(int i=0;i<3;i++){

            cartesianVelocityTargetMsg.points[0].velocities[i] = cartesianXYZTarget(i);//cartesianXYZTarget(i)

            cartesianVelocityTargetMsg.points[0].velocities[i+3] = cartesianRPYTarget(i);//cartesianRPYTarget(i)

            if(STATE ==ROTATING)
                cartesianVelocityTargetMsg.points[0].velocities[i+3]*=-1;

            currentSpeedRecordMsg.angle[i] = rawForce(i);
            currentSpeedRecordMsg.angle[i+3] = currentCartesianForce(i);
        }
        pub_speed.publish(cartesianVelocityTargetMsg);
        pub_record.publish(currentSpeedRecordMsg);
        return true;
    }

    void run(){
        init();
        while(1){
            STATE =APPROACHING;
            bool initDone =initRobotState(); //默认位置
            if(!initDone){
                ROS_INFO("robot init err, system shut down");
                return;
            }
            ROS_INFO("robot inited");
            int pause;
            ROS_INFO("push any key to continue :");
            std::cin>>pause;
            if(pause==3)
                STATE=PULLBACK;
            int countTime=0;
            bool sensorBias =true; //当为true时候,采集bias的值;为false时候进入控制模式
            while(ros::ok()){
                ros::spinOnce();
                //checkState();
                ROS_INFO_STREAM("current state "<<STATE);
                if(STATE ==STOP || STATE == PULLBACK){ 
                    break;//跳出控制循环
                }
                else{
                    ROS_INFO("HERE");
                    if(sensorBias && countTime<10){
                        if(rawCartesianForce(0)<0.00001 && rawCartesianForce(0)>-0.00001){
                            ROS_INFO("force is zero");
                        }
                        else{
                            ROS_INFO("MEAN FILTER");
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
                        ROS_INFO("CONTROLLED");
                    }
                }
                ROS_INFO("SEND MESSAGE");
                publicControlMessage();
                loop_rate.sleep();
            }
            if(STATE==PULLBACK){
                pull_back();
            }

        }
        ros::waitForShutdown();
    }

    inline void getParam(){
        nh_.param<double>("imp_mf/x",MF(0,0),100);
        nh_.param<double>("imp_mf/y",MF(1,1),100);
        nh_.param<double>("imp_mf/z",MF(2,2),100);

        nh_.param<double>("imp_bf/x",BF(0,0),2000);
        nh_.param<double>("imp_bf/y",BF(1,1),2000);
        nh_.param<double>("imp_bf/z",BF(2,2),2000);

        nh_.param<double>("imp_mt/x",MT(0,0),100);
        nh_.param<double>("imp_mt/y",MT(1,1),100);
        nh_.param<double>("imp_mt/z",MT(2,2),100);

        nh_.param<double>("imp_bt/x",BT(0,0),2000);
        nh_.param<double>("imp_bt/y",BT(1,1),2000);
        nh_.param<double>("imp_bt/z",BT(2,2),2000);

        nh_.param<double>("pid_pf/x",PF(0,0),0);
        nh_.param<double>("pid_pf/y",PF(1,1),0);
        nh_.param<double>("pid_pf/z",PF(2,2),0);

        nh_.param<double>("pid_df/x",DF(0,0),0);
        nh_.param<double>("pid_df/y",DF(1,1),0);
        nh_.param<double>("pid_df/z",DF(2,2),0);

        nh_.param<double>("pid_pt/x",PT(0,0),0);
        nh_.param<double>("pid_pt/y",PT(1,1),0);
        nh_.param<double>("pid_pt/z",PT(2,2),0);

        nh_.param<double>("pid_dt/x",DT(0,0),0);
        nh_.param<double>("pid_dt/y",DT(1,1),0);
        nh_.param<double>("pid_dt/z",DT(2,2),0);

        nh_.param<double>("impedance_force_target/x",cartesianForceTarget(0),0);
        nh_.param<double>("impedance_force_target/y",cartesianForceTarget(1),0);
        nh_.param<double>("impedance_force_target/z",cartesianForceTarget(2),0);

        nh_.param<double>("impedance_torque_target/x",cartesianTorqueTarget(0),0);
        nh_.param<double>("impedance_torque_target/y",cartesianTorqueTarget(1),0);
        nh_.param<double>("impedance_torque_target/z",cartesianTorqueTarget(2),0);

        nh_.param<double>("start_position/x1",start_position[0],-90.);
        nh_.param<double>("start_position/x2",start_position[1],-90.);
        nh_.param<double>("start_position/x3",start_position[2],-90.);
        nh_.param<double>("start_position/x4",start_position[3],-90.);
        nh_.param<double>("start_position/x5",start_position[4],90.);
        nh_.param<double>("start_position/x6",start_position[5],0.);

    }

};

int main(int argc,char** argv){

    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ImpedanceControl imp_;
    imp_.run();
    //imp_.PTHybirControl();
    return 0;
}