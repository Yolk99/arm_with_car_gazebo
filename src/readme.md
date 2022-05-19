# arm_with_car_gazebo

NOTE：
    使用ubuntu20.04 + ROS Noetic编写，其他版本可能要修改代码
    xacro模型是arm_with_car_description/urdf/arm.xacro
    ZED深度相机使用的是D435的gazebo插件（视角和可视距离还没改）
    需要下载门的gazebo模型，下载地址https://data.nvision2.eecs.yorku.ca/3DGEMS/ ，或者将door_3d.zip解压到/usr/share/gazebo-**/models目录下
    需要搭建yolov5环境


查看模型
    rviz：
    roslaunch arm_with_car_description display.launch
    gazebo：
    roslaunch arm_with_car_description gazebo.launch

gazebo仿真
    不带机械臂、不带导航仿真：
    roslaunch arm_with_car_gazebo view_arm_autolabor_with_laser_gazebo.launch
    带机械臂、不带导航仿真：
    roslaunch arm_with_car_gazebo arm_bringup_moveit.launch
    gmapping建图：
    roslaunch slam_navigation gmapping_demo.launch
    完整仿真：
    roslaunch arm_with_car_gazebo bringup_all.launch

识别门把手
    识别，需要注意的是image_topic的话题和weights_name的路径(我用的绝对路径，要更改，相对路径在yolov5_deepsort/scripts/yolov5/weights/best.pt)：
    roslaunch yolov5_deepsort detector.launch
    显示，话题为detections_image_topic
    rqt_image_view
