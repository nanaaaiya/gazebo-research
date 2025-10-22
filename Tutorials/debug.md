export PX4_SIM_SPEED_FACTOR=0.4
cd ~/ctuav_gazebo-research/PX4-Autopilot/

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-8,8,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0


PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="8,8,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1


ros2 run px4_ros_com offboard_multi --ros-args -p ns:=/px4_1 -p target_system:=2 -p hover_x:=0.0 -p hover_y:=-16.0 -p hover_z:=-5.0  


ros2 run px4_ros_com offboard_multi --ros-args -p target_system:=1 -p hover_x:=0.0 -p hover_y:=16.0 -p hover_z:=-5.0


k nhất thiết phải đâm vào nhau mới là va chạm
có thể do cái collision box lớn nên k chạm?
-> define va chạm là trong khoảng cách 1m


có thể  thêm 1 function tính khoảng cách giwuax 2 con drone, nếu dưới 1m là xác định va chạm



- offboard + reach waypoints: offboard_multi
- collision avoidance: offboard_swarm


ros2 run px4_ros_com offboard_swarm_monitor --ros-args -r __node:=swarm_monitor_1


ros2 run px4_ros_com offboard_swarm_monitor --ros-args -r __node:=swarm_monitor_2