source devel/setup.bash

roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=192.168.0.33

roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch

roslaunch ur3_moveit_config moveit_rviz.launch config:=true

Any information, please contact with inksci@qq.com
