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
        arm.set_goal_joint_tolerance(0)

        while 1:
            print('\033[34m【info】***任意关节角度输入10000.0，结束进程***')
            print('\033[34m【info】****单位：角度****')

            #a = float(input("关节1角度：")) * 3.1415926 /180
            a = 90 * 3.1415926 /180
            if a == 10000.0  * 3.1415926 /180:
                break
            #b = float(input("关节2角度：")) * 3.1415926 /180
            b = 50 * 3.1415926 /180
            if b == 10000.0  * 3.1415926 /180:
                break
            #c = float(input("关节3角度：")) * 3.1415926 /180
            c = -50 * 3.1415926 /180
            if c == 10000.0  * 3.1415926 /180:
                break
            #d = float(input("关节4角度：")) * 3.1415926 /180
            d = 0 * 3.1415926 /180
            if d == 10000.0  * 3.1415926 /180:
                break
            #e = float(input("关节5角度：")) * 3.1415926 /180
            e = 0 * 3.1415926 /180
            if e == 10000.0  * 3.1415926 /180:
                break
            #f = float(input("关节6角度：")) * 3.1415926 /180
            f = 90 * 3.1415926 /180
            if f == 10000.0  * 3.1415926 /180:
                break
            


         
            # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
            joint_positions = [a, b, c, d, e, f]
            arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
            print('\033[33m执行中...')
            arm.go()
            print('\033[32m执行完毕！')
            print("  ")
            rospy.sleep(1)

            break

        print('\033[31m结束进程')
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
