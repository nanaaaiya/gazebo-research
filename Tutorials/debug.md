```bash
export PX4_SIM_SPEED_FACTOR=0.4
cd ~/ctuav_gazebo-research/PX4-Autopilot/

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-8,8,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0


PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="8,8,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1


cd ~/ctuav_gazebo-research/Custom_QGC/qgroundcontrol
./build/Debug/QGroundControl


MicroXRCEAgent udp4 -p 8888


source install/setup.bash
ros2 launch mavros multi_uas.launch


ros2 run px4_ros_com offboard_multi --ros-args -p ns:=/px4_1 -p target_system:=2 -p hover_x:=0.0 -p hover_y:=-16.0 -p hover_z:=-5.0  


ros2 run px4_ros_com offboard_multi --ros-args -p target_system:=1 -p hover_x:=0.0 -p hover_y:=16.0 -p hover_z:=-5.0


ros2 run px4_ros_com offboard_multi --ros-args -p ns:=/px4_1 -p target_system:=2 -p hover_x:=0.0 -p hover_y:=-9.0 -p hover_z:=-5.0 


ros2 run px4_ros_com offboard_multi --ros-args -p target_system:=1 -p hover_x:=0.0 -p hover_y:=9.0 -p hover_z:=-5.0


k nhất thiết phải đâm vào nhau mới là va chạm
có thể do cái collision box lớn nên k chạm?
-> define va chạm là trong khoảng cách 1m


có thể  thêm 1 function tính khoảng cách giwuax 2 con drone, nếu dưới 1m là xác định va chạm



- offboard + reach waypoints: offboard_multi
- collision avoidance: offboard_swarm


ros2 run px4_ros_com offboard_swarm_monitor --ros-args -r __node:=swarm_monitor_1


ros2 run px4_ros_com offboard_swarm_monitor --ros-args -r __node:=swarm_monitor_2



ros2 run px4_ros_com collision_avoidance

ros2 run px4_ros_com offboard_switcher


Consider:

- tần số  xử lí

Note: k phải cái nào cũng cùng 1 tần số

- bán kính trc khi đụng nhau

- tốc độ bay: 10 - 15m/s trong trạng thái ổn định

- tốc độ xử lí

Note: nếu bay với vận tốc 20m/s với khoảng cách 2m để  alert thành ra chỉ có 0.1s để xử lí 



- velocity of drones:
- bans kinh: 2m
-




colcon build --symlink-install --packages-select px4_msgs mavlink mavros_msgs px4_ros_com



pkill -9 px4
pkill -9 gz
pkill -9 micrortps_agent


ctuav@ctuav:~/ctuav_gazebo-research$ ros2 run px4_ros_com offboard_multi --ros-args -p target_system:=1 -p hover_x:=0.0 -p hover_y:=9.0 -p hover_z:=-5.0
[INFO] [1761901809.836517589] [offboard_multi]: Using namespace '' -> topics: /fmu/in/offboard_control_mode , /fmu/in/trajectory_setpoint , /fmu/in/vehicle_command
terminate called after throwing an instance of 'std::out_of_range'
  what():  basic_string::substr: __pos (which is 1) > this->size() (which is 0)
[ros2run]: Aborted



unresolved problems:

- already in OFFBOARD mode when flying to assigned goals, so no need to switch to this mode again in offboard_switcher -> done

- check if the collision avoidance successfully sends the waypoints and list all of them
-> done

- check if offboard_swithcer receives the changing waypoints -> done

- version tracking problem ????????????????????




```
