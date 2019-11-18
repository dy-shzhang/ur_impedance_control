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
    //robot_model_loader("robot_description")
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::MoveItErrorCode success; 
    group.setStartState(*group.getCurrentState());

    group.setGoalJointTolerance(0.01);
    group.setMaxVelocityScalingFactor(0.1);

    geometry_msgs::Pose target_pose; 
    std::vector<geometry_msgs::Pose> path;

    target_pose.orientation.w =1.000000; 
    target_pose.orientation.x = 0.000000; 
    target_pose.orientation.y = 0.000000; 
    target_pose.orientation.z = 0.000000; 
//末端姿态三维坐标
    target_pose.position.x = 0.10; 
    target_pose.position.y = 0.50;
    target_pose.position.z = 0.40; 

     path.push_back(target_pose);

     target_pose.position.x +=0.1;
     target_pose.position.y -=0.1;
     target_pose.position.z +=0.1;
     path.push_back(target_pose);

     target_pose.position.x -=0.2;
     target_pose.position.y -=0.1;
     target_pose.position.z +=0.1;
     path.push_back(target_pose);

     moveit_msgs::RobotTrajectory trajectory;

     double fraction = group.computeCartesianPath(path,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);

    //group.setPoseTarget(target_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan; 
    // success =group.plan(plan); 
    // if(success){
    //     group.execute(plan);
    // }
    return 0;
}

