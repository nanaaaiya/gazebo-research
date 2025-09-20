# gazebo-research

---

## 1. Get odometry data from one drone

### Start the Client (Run PX4-Gazebo simulation):

The PX4 simulator starts the uXRCE-DDS client automatically, connecting to UDP port 8888 on the local host.

```bash
cd ctuav_gazebo-research/PX4-Autopilot/
make px4_sitl gz_x500
```

More vehicle and world models: https://docs.px4.io/v1.15/en/sim_gazebo_gz/


### Start the agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

> **Note:** Only one agent is allowed per connection channel.

### Start QGroundControl and start drone's missions:

```bash
./QGroundControl.AppImage
```


## Launch odometry data listener node:

```bash
ros2 run px4_ros_com vehicle_odometry_listener
```

If this is working you should see data being printed on the terminal/console where you launched the ROS listener:

```bash
RECEIVED VEHICLE ODOMETRY DATA
==================================
timestamp: 1758331893367780
timestamp_sample: 1758331893367780
pose_frame: 1
position: x=27.911818 y=96.151924 z=-10.104164
q (quat): [0.799043, 0.020722, -0.021116, 0.600545]
velocity_frame: 1
velocity: vx=1.392928 vy=4.826545 vz=0.003994
angular_velocity (body): [-0.004274, -0.001851, 0.001207]
position_variance[0..2]: 0.011479, 0.011483, 0.027872
orientation_variance[0..2]: 0.000033, 0.000029, 0.000567
velocity_variance[0..2]: 0.003759, 0.003765, 0.002967
reset_counter: 13
quality: 0
```

---


