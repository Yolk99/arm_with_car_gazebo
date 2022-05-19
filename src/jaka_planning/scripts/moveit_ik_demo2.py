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
               
        
         
        while 1:
            print('\033[34m【info】***请输入x、y或z，输入o结束进程***')
            a = (input("方向选择："))
            if a == 'x':
                axis = 0
                print('\033[34m【info】***正为前，单位m***')
                distance = float(input("向前："))
            elif a == 'y':
                axis = 1
                print('\033[34m【info】***正为左，单位m***')
                distance = float(input("向左："))
            elif a == 'z':
                axis = 2
                print('\033[34m【info】***正为上，单位m***')
                distance = float(input("向上："))
            elif a == 'o':
                print('\033[31m【info】***结束进程***')
                break
            
            arm.shift_pose_target(axis, distance, end_effector_link)
            if a == 'x':
                print('\033[33m终端向前移动...')
            elif a == 'y':
                print('\033[33m终端向左移动...')
            elif a == 'z':
                print('\033[33m终端向上移动...')
            arm.go()
            print('\033[32m执行完毕！')
            print("  ")
            rospy.sleep(1)



           


        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    