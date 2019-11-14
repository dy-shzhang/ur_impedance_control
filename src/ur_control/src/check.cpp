/*
#include <moveit/move_group_interface/move_group_interface.h> 
#include<moveit_msgs/AttachedCollisionObject.h> 
#include<moveit_msgs/CollisionObject.h>
int main(int argc, char **argv) {
ros::init(argc, argv, "move_group_interface_tutorial"); 
ros::NodeHandle node_handle; 
ros::AsyncSpinner spinner(1); 
spinner.start();
moveit::planning_interface::MoveGroupInterface group("manipulator");
moveit::planning_interface::MoveItErrorCode success; 

//设置初始位置
group.setStartState(*group.getCurrentState()); 

//设置约束 

moveit_msgs::Constraints endEffector_constraints; 
moveit_msgs::OrientationConstraint ocm; 

ocm.link_name= "ee_link";//需要约束的链接 
ocm.header.frame_id = "base_link";//基坐标系

 //四元数约束
ocm.orientation.w = 1.0; //欧拉角约束 
ocm.absolute_x_axis_tolerance = 2*3.14;
ocm.absolute_y_axis_tolerance = 2*3.14; 
ocm.absolute_z_axis_tolerance = 2*3.14;
ocm.weight = 1.0;//此限制权重

endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表

group.setPathConstraints(endEffector_constraints);//设置约束 

//设置抓取目标点
geometry_msgs::Pose target_pose; 
//末端姿态四元数 
target_pose.orientation.w =1.000000; 
target_pose.orientation.x = 0.000000; 
target_pose.orientation.y =0.000000; 
target_pose.orientation.z = 0.000000; 
//末端姿态三维坐标
target_pose.position.x = 0.40; 
target_pose.position.y = -0.40;
target_pose.position.z = 0.50; 
//进行运动规划 
group.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan; 
success =group.plan(plan); 
group.clearPathConstraints();//清除约束 
if(success)
    group.move();

//运动规划输出
//ROS_INFO("Visualizing plan (stateCatch pose) %s",success ==moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");

//if (success ==moveit_msgs::MoveItErrorCodes::SUCCESS) 
//    group.execute(plan); 
ros::shutdown();
return 0; 
}

*/
#include <moveit/move_group_interface/move_group_interface.h> 
#include<moveit_msgs/AttachedCollisionObject.h> 
#include<moveit_msgs/CollisionObject.h>
#include<moveit/planning_request_adapter/planning_request_adapter.h> 
#include<moveit/trajectory_processing/iterative_time_parameterization.h>
#include<geometry_msgs/PointStamped.h> 
#include<math.h>
#include<vector>

using namespace std;

int main(int argc, char **argv) {
ros::init(argc, argv, "move_group_interface_tutorial"); 
ros::NodeHandle node_handle; 
ros::AsyncSpinner spinner(1); 
spinner.start();
moveit::planning_interface::MoveGroupInterface group("robot");
moveit::planning_interface::MoveItErrorCode success; 
//设置初始位置
group.setStartState(*group.getCurrentState());
/*
//设置约束 moveit_msgs::Constraints
endEffector_constraints; moveit_msgs::OrientationConstraint ocm; 
ocm.link_name= "gripper";
//需要约束的链接 
ocm.header.frame_id = "base_link";
//基坐标系 

//四元数约束
ocm.orientation.w = 1.0; //欧拉角约束 
ocm.absolute_x_axis_tolerance = 0.1;
ocm.absolute_y_axis_tolerance = 0.1; 
ocm.absolute_z_axis_tolerance = 2*3.14;
ocm.weight = 1.0;//此限制权重
endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表

//设置运动路径 
std::vector<geometry_msgs::Pose> waypoints; geometry_msgs::Pose target_pose; 
target_pose.orientation.w = 0.000000; 
target_pose.orientation.x =0.000000; 
target_pose.orientation.y = 0.000000; 
target_pose.orientation.z =0.000000; 

target_pose.position.x = 0.000789; 
target_pose.position.y =-0.089177; 
target_pose.position.z = 0.247533; 
waypoints.push_back(target_pose);
target_pose.orientation.w = 0.000000; 
target_pose.orientation.x = 0.000000;
target_pose.orientation.y = 0.000000; 
target_pose.orientation.z = 0.000000;
target_pose.position.x = -0.020000; 
target_pose.position.y = -0.005000;
target_pose.position.z = 0.350000; 
waypoints.push_back(target_pose); 

//进行运动规划
moveit_msgs::RobotTrajectory trajectory_msg; 
double fraction = group.computeCartesianPath( waypoints, 0.01, // eef_step, 
                                        0.0, //jump_threshold 
                                        trajectory_msg, 
                                        endEffector_constraints,//constraints 
                                        false);
ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction *100.0); 

group.clearPathConstraints(); // The trajectory needs to be modified so it will include velocities as well.
// First to create a RobotTrajectory object
robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(),"hand"); 

// Second get a RobotTrajectory from trajectory
rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg); 
// Thrid
create a IterativeParabolicTimeParameterization object
trajectory_processing::IterativeParabolicTimeParameterization iptp; 

// Fourth
compute computeTimeStamps bool ItSuccess = iptp.computeTimeStamps(rt);
ROS_INFO("Computed time stamp %s",ItSuccess?"SUCCEDED":"FAILED"); 
// Get
RobotTrajectory_msg from RobotTrajectory
rt.getRobotTrajectoryMsg(trajectory_msg); //输出运动
moveit::planning_interface::MoveGroupInterface::Plan plan; plan.trajectory_ =trajectory_msg; group.execute(plan); 
*/
ros::shutdown(); 
return 0; 
} 

std::vector<double> EulerToQuaternion(std::vector<double>& euler){
    std::vector<double>Quaternion(4,0.0);
    if(euler.size()!=3){
        return Quaternion;
    }
    double rx = euler[0];
    double ry = euler[1];
    double rz = euler[2];
    Quaternion[0] = cos(rx/2.)*cos(ry/2.)*cos(rz/2.)+sin(rx/2.)*sin(ry/2.)*sin(rz/2.);
    Quaternion[1] = sin(rx/2.)*cos(ry/2.)*cos(rz/2.)-cos(rx/2.)*sin(ry/2.)*sin(rz/2.);
    Quaternion[2] = cos(rx/2.)*sin(ry/2.)*cos(rz/2.)+sin(rx/2.)*cos(ry/2.)*sin(rz/2.);
    Quaternion[3] = cos(rx/2.)*cos(ry/2.)*sin(rz/2.)-sin(rx/2.)*sin(ry/2.)*cos(rz/2.);
    ROS_INFO("Euler converts to Quaternion");
    return Quaternion;
}
