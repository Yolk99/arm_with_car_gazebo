//letsmoveit.cpp
#include <moveit/move_group_interface/move_group_interface.h>
 
int main(int argc, char **argv)
{
  //初始化节点
  ros::init(argc, argv, "jaka_moveit");

  ros::NodeHandle node_handle;
  //引入多线程 
  ros::AsyncSpinner spinner(1);
  //开启多线程
  spinner.start();
 
 //初始化需要使用move group控制的机械臂中的arm group
  moveit::planning_interface::MoveGroupInterface arm("arm");//arm对应moveit中选择的规划部分

  //获取终端link的名称
  std::string end_effector_link = arm.getEndEffectorLink();
 
  //设置目标位置所使用的参考坐标系
  std::string reference_frame = "base_footprint";
  arm.setPoseReferenceFrame(reference_frame);
 
  //当运动规划失败后，允许重新规划
  arm.allowReplanning(true);


  arm.setGoalPositionTolerance(0.01);
  arm.setGoalOrientationTolerance(0.1);

  // 控制机械臂先回到初始化位置
  arm.setNamedTarget("home");
  arm.move(); //规划+移动
  sleep(1);  //停1s
 
  // 设置发送的数据，对应于moveit中的拖拽
  geometry_msgs::Pose target_pose1;


  
  float x_target, y_target, z_target, roll_target = 0.f, pitch_target = 0.f, yaw_target = 0.f;
  while(1)
  {
    
    printf("\033[34m""***任意坐标输入9.0结束进程***\r\n"); //蓝
    printf("\033[34m""输入目标三轴坐标,单位m："); //蓝
    scanf("%f %f %f",&x_target, &y_target ,&z_target);

    if(x_target == 9.0 || y_target == 9.0 || z_target == 9.0)
    {
      printf("\033[31m""结束进程！\r\n");
      break;
    }

    target_pose1.orientation.x = 0.0;
    target_pose1.orientation.y = 0.0;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 1.0;

    target_pose1.position.x = x_target;
    target_pose1.position.y = y_target;
    target_pose1.position.z = z_target;
  
    arm.setPoseTarget(target_pose1);
    
    printf("\033[33m""计算中...\r\n");  //黄
  
  
    // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(my_plan);
  
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
  
    //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
    if(success)
    {
      printf("\033[32m""路径规划成功！\r\n"); //绿
      printf("\033[33m""执行中...\r\n");  //黄
      arm.execute(my_plan);
      printf("\033[32m""执行成功！\r\n");  //绿
      printf("   \r\n");
    }
      
    else
    {
      printf("\033[31m""路径规划失败！\r\n"); //红
      printf("   \r\n");
      printf("\033[33m""请重设目标点\r\n"); //黄
    }
      
    

  }

  ros::shutdown(); 
  return 0;
}
