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
    ros::Publisher pub_position;
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
    trajectory_msgs::JointTrajectory cartesianPositionTargetMsg;
    
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


    Eigen::Vector3d formerCartesianForce; //记录力,世界坐标系下
    Eigen::Vector3d formerCartesianTorque;//当前末端笛卡尔力矩
    double filterAlpha; 

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
        pub_speed =nh_.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed",1);// /ur_driver/joint_speed
        pub_position =nh_.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/tcp_position",1);
        pub_record =nh_.advertise<ur_control::jointSpeedRecord>("/joint_speed_record",1);// /joint_speed_record

        cartesianVelocityTargetMsg.points.resize(1);
        cartesianVelocityTargetMsg.points[0].velocities.resize(6);
        cartesianVelocityTargetMsg.points[0].accelerations.resize(1);
        currentSpeedRecordMsg.angle.resize(6);

        cartesianVelocityTargetMsg.points[0].accelerations[0]=3;

        cartesianPositionTargetMsg.points.resize(1);
        cartesianPositionTargetMsg.points[0].velocities.resize(1);
        cartesianPositionTargetMsg.points[0].velocities[0]=0.25;
        cartesianPositionTargetMsg.points[0].accelerations.resize(1);
        cartesianPositionTargetMsg.points[0].accelerations[0]=1.4;
        cartesianPositionTargetMsg.points[0].positions.resize(6);


        PI=3.141592653589793238;
        endEffectOffset<<0.0,0.3232,0.0; //+ 0,0.25,0 是手爪末端位置
        //endEffectOffset<<0.0,0.242,0.0;
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

        formerCartesianForce<<0,0,0;
        formerCartesianTorque<<0,0,0;
        filterAlpha=0.5;

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
    
        for(int i=0;i<3;i++){
            if(rawCartesianForce(i)>200 ||rawCartesianForce(i)<-200){
                rawCartesianForce = formerCartesianForce;
                break;
            }
        }

        rawCartesianForce = filterAlpha*rawCartesianForce + (1-filterAlpha)*formerCartesianForce; //低通滤波器
        formerCartesianForce = rawCartesianForce;
        //ROS_INFO_STREAM(formerCartesianForce);

        Eigen::Vector3d tmpTorque(msg.tx,msg.ty,msg.tz);
        rawTorque = tmpTorque;
        rawCartesianTorque = world2force*tmpTorque;

        for(int i=0;i<3;i++){
            if(rawCartesianTorque(i)>200 ||rawCartesianTorque(i)<-200){
                rawCartesianTorque = formerCartesianTorque;
                break;
            }
        }
        

        rawCartesianTorque = filterAlpha*rawCartesianTorque + (1-filterAlpha)*formerCartesianTorque; //低通滤波器
        formerCartesianTorque = rawCartesianTorque;
    }

    void deadZoom(Eigen::Vector3d& matrx,double range){
        for(int i=0;i<3;i++){
            if(matrx(i)<range && matrx(i)>range*-1)
                matrx(i) = 0.;
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
        cartesianXYZTarget =DF*(errInForce-perErrInForce)+PF*errInForce;
        cartesianXYZTarget =velocity2world*cartesianXYZTarget;
        perErrInForce =errInForce;  
        for(int i=0;i<3;i++){
            if(errInForce(i)<0.5 && errInForce(i)>-0.5)
                cartesianXYZTarget(i)=0.;
        }


        errInTorque = currentCartesianTorque - cartesianTorqueTarget;
        cartesianRPYTarget = DT*(errInTorque - perErrInTorque)+PT*errInTorque;
        cartesianRPYTarget =velocity2world*cartesianRPYTarget;
        perErrInTorque =errInTorque;
        // for(int i=0;i<3;i++){
        //     if(errInTorque(i)<0.5 && errInTorque(i)>-0.5)
        //         cartesianRPYTarget(i)=0.;
        // }

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

    void impendancePosition(){
        //tcp 的在tool0 link 中,和wrist_3_link rpy=[-90,0,0];
        init();
        bool initDone =initRobotState(); //默认位置
        if(!initDone){
            ROS_INFO("robot init err, system shut down");
            return;
        }
        ROS_INFO("robot inited");
        int pause;
        ROS_INFO("push any key to continue :");
        std::cin>>pause;

         Eigen::Matrix3d wrist2tool;
        wrist2tool<< 1,0,0,0,0,1,0,-1,0;
        cartesianPositionTargetMsg.points[0].accelerations[0]=0.5;
        cartesianPositionTargetMsg.points[0].velocities[0]=0.1;

        while(ros::ok()){
            ros::spinOnce();
            Eigen::Vector3d tcpPosition = velocity2world*getEndPosition();
            Eigen::Vector3d tcpEuler = getEulerAngle(); // 获取欧拉角
            Eigen::Matrix3d rotation = tf.rotation()*wrist2tool;
            Eigen::AngleAxisd rotation_vector (velocity2world*rotation);
            Eigen::Vector3d tcporientation = rotation_vector.angle()*rotation_vector.axis().transpose();
            std::cout<<"vector" <<tcporientation<< std::endl;
            std::cout<<" position "<<tcpPosition << std::endl;
            for(int i=0;i<3;i++){
                cartesianPositionTargetMsg.points[0].positions[i] = tcpPosition(i);
                cartesianPositionTargetMsg.points[0].positions[i+3] = tcporientation(i);
            }
 
            pub_position.publish(cartesianPositionTargetMsg);
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
        //ROS_INFO_STREAM(tf.rotation());
        return currentCartesianEndEffortPositon;
    }

    Eigen::Vector3d getEulerAngle(){
        // tcp oriention;
        Eigen::Matrix3d wrist2tool;
        wrist2tool<< 1,0,0,0,0,1,0,-1,0;
        Eigen::Matrix3d rotation = tf.rotation()*wrist2tool;
        Eigen::Vector3d currentCartesianEndEffortOriention = rotation.eulerAngles(2,1,0);

        return currentCartesianEndEffortOriention;
    }

    Eigen::Vector3d Rodrigues(Eigen::Matrix3d R){
        double theta = acos((R.trace() - 1) * 0.5);	//待求的旋转向量的模长

        Eigen::Matrix3d Right = (R - R.transpose()) * 0.5 / sin(theta);
        Eigen::Vector3d r;	//待求的旋转向量的单位向量
        r[0] = (Right(2,1) - Right(1,2))*0.5;
        r[1] = (Right(0,2) - Right(2,0))*0.5;
        r[2] = (Right(1,0) - Right(0,1))*0.5;
        ROS_INFO_STREAM("theta"<<theta);
        ROS_INFO_STREAM("axis "<< r);
        ROS_INFO_STREAM("vector" << theta*r);
        return r;
    }
    
    void checkState(){ //判断插孔状态
        Eigen::Vector3d currentCartesianEndEffortPositon = getEndPosition();

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
        cartesianVelocityTargetMsg.points[0].accelerations[0]=1;
        for(int i=0;i<3;i++){

            cartesianVelocityTargetMsg.points[0].velocities[i] = cartesianXYZTarget(i);//cartesianXYZTarget(i)

            cartesianVelocityTargetMsg.points[0].velocities[i+3] = cartesianRPYTarget(i);//cartesianRPYTarget(i)

            if(STATE ==ROTATING)
                cartesianVelocityTargetMsg.points[0].velocities[i+3]*=-1;

            currentSpeedRecordMsg.angle[i] = cartesianRPYTarget(i);
            currentSpeedRecordMsg.angle[i+3] = currentCartesianTorque(i);
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
                //ROS_INFO_STREAM("current state "<<STATE);
                if(STATE ==STOP || STATE == PULLBACK){ 
                    break;//跳出控制循环
                }
                else{
                    if(sensorBias && countTime<10){
                        if(rawCartesianForce(0)<0.00001 && rawCartesianForce(0)>-0.00001){
                                ROS_INFO("HERE");
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
                    }
                }

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
    //imp_.run();
    //imp_.PTHybirControl();
    imp_.impendancePosition(); //movel()不好用啊
    return 0;
}