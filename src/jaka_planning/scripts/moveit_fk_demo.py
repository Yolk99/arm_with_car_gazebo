#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        

        
        # 设置机械臂的允许误差值
        arm.set_goal_joint_tolerance(0.1)

        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)
        
        a = float(input("轴1角度："))
        b = float(input("轴2角度："))
        c = float(input("轴3角度："))
        d = float(input("轴4角度："))
        e = float(input("轴5角度："))
        f = float(input("轴6角度："))
        print("执行中...")


         
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [a, b, c, d, e, f]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        print("执行完毕！")
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
