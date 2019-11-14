#include <string>
#include <vector>
#include <ros/ros.h>
#include <wukongii_arm/jointTarget.h>


//moveit 发来的trajectory 10ms 一个点 position + velocity
//一个电机一个参数

//电机控制层面

class jointBase{
public:
    double currentPosition;
    double currentVelocity;
    double currentTorque;

    double targetPosition;
    double targetVelocity;
    double targetTorque;
    std::string jointName_;
private:
    double P_,I_,D_;
public:
    jointBase(std::string name):jointName_(name){
        targetTorque =0.;
        P_=100.;
        I_=0.;
        D_=1.;
        ros::param::get(jointName_+"_P",P_);
        ros::param::get(jointName_+"_I",I_);
        ros::param::get(jointName_+"_D",D_);
    };

    void updateTarget(double& newTargetPosition, double& newTargetVelocity){
        targetPosition = newTargetPosition;
        targetVelocity = newTargetVelocity;

    };
    void updateState(double& newPosition, double& newVelocity,double& newTorque){
        currentPosition = newPosition;
        currentVelocity = newVelocity;
        currentTorque = newTorque;
    };

    void pidControl(double forwardTorque=0){ //forwardTorque =0. 时候可用增量式PID
        targetTorque = forwardTorque + P_*(targetPosition - currentPosition)+D_*(targetVelocity - currentVelocity);
    };
};

class baseControl{
public:
    std::vector<double> newTargetPositions;
    std::vector<double> newTargetVelocities;
    std::vector<double> newTargetTorque; //需要下发的目标力矩
    ros::NodeHandle nh_;
    ros::Subscriber sub_target;
    ros::Rate loop_rate;
    int renewTargetHZ; //更新目标的频率
    int numberPerArm;
    std::vector<jointBase*>larm,rarm;
    wukongii_arm::jointTarget targetMsgRecord;

    baseControl(int mainLoopRate,int subLoopRate,int jointNumber=4):loop_rate(mainLoopRate),
        numberPerArm(jointNumber),renewTargetHZ(subLoopRate){
        sub_target =nh_.subscribe("/target_info",1,&baseControl::renewTarget,this);
        for(int i=0;i<numberPerArm;i++){
            jointBase* tmp =new jointBase ("larm"+std::to_string(i+1)+"joint");
            larm.push_back(tmp);
        }
        for(int i=0;i<numberPerArm;i++){
            jointBase* tmp =new jointBase ("rarm"+std::to_string(i+1)+"joint");
            rarm.push_back(tmp);
        }
        newTargetPositions.resize(2*numberPerArm,0.);
        newTargetVelocities.resize(2*numberPerArm,0.);
    };

    void renewTarget(const wukongii_arm::jointTarget& msg){ //更新目标位置,速度信息
        wukongii_arm::jointTarget tmpTargetmsg = msg;
        if(tmpTargetmsg.positions.size()!=2*numberPerArm)
            tmpTargetmsg = targetMsgRecord;
        for(int i=0;i<numberPerArm;i++){
            larm[i]->updateTarget(tmpTargetmsg.positions[i],tmpTargetmsg.velocities[i]);
            rarm[i]->updateTarget(tmpTargetmsg.positions[i+numberPerArm],tmpTargetmsg.velocities[i+numberPerArm]);
        }
    };



    void run(){


    };

};

int main(int argc,char** argv){
    ros::init(argc, argv,"joint_control_base",ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int count =0;
    while(ros::ok()){
        if(count == 0){
            ros::spinOnce(); //接收新的目标位置
        }
    }
    return 0;
}