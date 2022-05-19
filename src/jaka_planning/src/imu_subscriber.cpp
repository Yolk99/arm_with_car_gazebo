#include <ros/ros.h> //ros标准库头文件
#include <iostream> //C++标准输入输出库
#include <std_msgs/String.h> //包含标准信息的字符串消息头文件
#include <vector>
#include <sensor_msgs/Imu.h>

using namespace std;

struct EulerAngles
{
    double roll, pitch, yaw;
};

bool Quaternion2EulerAngles(double w, double x, double y, double z, double &roll, double &pitch, double &yaw)
{
    double absQ2 = w * w + x * x + y * y + z * z;
    if (absQ2 > 1.2 || absQ2 < 0.8)
    {
        return false;
    }
    else
    {
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (abs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // 如果超出范围，使用90度
        else
            pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
        return true;
    }
}

void IMU_Callback(const sensor_msgs::Imu& imu_msg)
{
    EulerAngles angles;
    Quaternion2EulerAngles(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z,
                           angles.roll, angles.pitch, angles.yaw);

    ROS_INFO("EulerAngles is: roll = [%lf]; pitch = [%lf]; yaw =  [%lf]", angles.roll, angles.pitch, angles.yaw); //打印消息
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu", 10, IMU_Callback);

    ros::spin();
}