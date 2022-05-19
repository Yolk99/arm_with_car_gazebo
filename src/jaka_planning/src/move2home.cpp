
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
using namespace std;
int main(int argc, char **argv)
{

    //ros初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "moveit_fk_demo");
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();
 
    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    //设置机械臂运动的允许误差
    arm.setGoalJointTolerance(0.01);
    gripper.setGoalJointTolerance(0.01);
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    gripper.setMaxAccelerationScalingFactor(0.5);
    gripper.setMaxVelocityScalingFactor(0.5);
 
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move(); //规划+运动
    sleep(1);

    gripper.setNamedTarget("closed");
    gripper.move();
    sleep(1);

    gripper.setNamedTarget("open");
    gripper.move();
    sleep(1);

    //关闭并退出
    ros::shutdown(); 
 
    return 0;
}
