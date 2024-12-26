# Rikkert setup

## Getting started -- Making connection with onboard PC

A RDP protocol is running on the onboard PC together with the VNC protocol such that the PC can be remotely accessed. The recommended RDP viewer is [Remmina](https://remmina.org/). With the following information, one should be able to access the onboard PC:

| Type | Info |
| -------- | ------- |
| IP Address onboard PC | 192.168.69.20 |
| RDP login name | acro |
| RDP login password | acro |

## Starting the mobile robot

Navigate to the ```rikkert_ws``` ROS2 workspace, and make sure it is sourced. Execute the following launch file to bringup the robot:

```
ros2 launch rikkert_support rikkert_bringup.launch.py
```

For visualisation purposes, the following launch file visualises the robot model and the state of the robot in RViz2:

```
ros2 launch rikkert_description view_rikkert_sim.launch.py
```

### Starting 3D LiDAR
To start and use the RoboSense 3D LiDAR:
```
ros2 launch rs_bpearl rs_bpearl.launch.py
```

### Services and interfaces
The mobile robot posesses three states: **running**, **error**, and **idle**, indicated by respectively a _green_, _red_, and _orange_ indicator light at the side of the robot platform. When an emergency button is pressed, the robot goes in the **error** state. When the emergency state button is manually released or disactivated, the system goes to the **idle** state, indicating with an _orange_ light. Now, a **reset service call** in ROS2 is needed to switch the robot to the **running** state. This service call can be called as follows:
```
ros2 service call /acknowledgement std_srvs/srv/Trigger
```

To reset the encoders of the mobile platform, and thus resetting the odometry, following service can be called:
```
ros2 service call /reset_encoders std_srvs/srv/Trigger
```

## Teleoperation of the mobile platform
When a joystick is inserted in the USB hub of the mobile platform, the following ROS2 node can be called to teleoperate the mobile platform:
```
ros2 run joy joy_node 
```
The following commands are used to control the mobile robot:

| Button | Action |
| -------- | ------- |
| RB | Deadman switch |
| RT | Forward acceleration |
| LT | Backward acceleration |
| Left Axis (Left) | Rotational left |
| Left Axis (Right) | Rotational right |

