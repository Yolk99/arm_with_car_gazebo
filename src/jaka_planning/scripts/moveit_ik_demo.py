#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_footprint'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        print('\033[33m回到home位置...')
        arm.go()
        print('\033[32m执行完毕！')
        print("  ")
        rospy.sleep(2)
               
 
         
        # 控制机械臂终端向下移动20cm
        arm.shift_pose_target(2, -0.2, end_effector_link)
        #                     ↑
        #                     0: x, 1: y, 2: z
        print('\033[33m终端向下移动20cm...')
        arm.go()
        print('\033[32m执行完毕！')
        print("  ")
        rospy.sleep(1)

        # 控制机械臂终端向上移动10cm
        arm.shift_pose_target(2, 0.1, end_effector_link)
        print('\033[33m终端向上移动10cm...')
        arm.go()
        print('\033[32m执行完毕！')
        print("  ")
        rospy.sleep(1)

        # 控制机械臂终端向前移动10cm
        arm.shift_pose_target(0, 0.1, end_effector_link)
        print('\033[33m终端向前移动10cm...')
        arm.go()
        print('\033[32m执行完毕！')
        print("  ")
        rospy.sleep(1)

        # 控制机械臂终端向左移动10cm
        arm.shift_pose_target(1, 0.1, end_effector_link)
        print('\033[33m终端向左移动10cm...')
        arm.go()
        print('\033[32m执行完毕！')
        print("  ")
        rospy.sleep(1)
  
        # 控制机械臂终端反向旋转90度
        #arm.shift_pose_target(3, -1.57, end_effector_link)
        #print('\033[33m终端反向旋转90度...')
        #arm.go()
        #print('\033[32m执行完毕！')
        #print("  ")
        #rospy.sleep(1)
           


        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    