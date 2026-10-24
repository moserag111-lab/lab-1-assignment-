# Lab 1 – ROS 2, Turtlesim, Actions & Services

## Team Members
| Name | Student ID |
|------|------------|
| Mohamed Naser Serag | 9799 |
| *(teammate 2 name)* | *(ID)* |
| *(teammate 3 name)* | *(ID)* |

Alexandria University – Faculty of Engineering – Design of Mechatronics Systems
Tested with: ROS 2 Humble / Ubuntu 22.04 (Python, rclpy)

## Packages
| Package | Type | Purpose |
|---|---|---|
| `lab1_interfaces` | ament_cmake | `msg/TargetPose` (x,y,yaw), `action/GoToPose`, `action/DrawSquare`, `srv/SetSpeedLimits` |
| `lab1_turtle_control` | ament_python | `turtle_controller` (go_to_pose action + speed params + `set_speed_limits` service), `reset_service` (`reset_turtle`), `square_server` (`draw_square` action) |
| `lab1_bringup` | ament_python | Launch files + `config/speed_params.yaml` |

## Build
```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-turtlesim python3-colcon-common-extensions
mkdir -p ~/lab1_ws/src && cd ~/lab1_ws/src
git clone <your-repo-url> .
cd ~/lab1_ws && rosdep install --from-paths src -y --ignore-src
colcon build && source install/setup.bash
```

## Run
```bash
ros2 launch lab1_bringup system.launch.py                       # turtlesim + all servers (R1,R6,R7)
ros2 launch lab1_bringup system.launch.py params_file:=/path/my.yaml
ros2 launch lab1_bringup square.launch.py                       # system + automatic square
```

## Try every requirement
```bash
# R2 interfaces
ros2 interface list | grep lab1_interfaces
ros2 interface show lab1_interfaces/msg/TargetPose
ros2 interface show lab1_interfaces/action/GoToPose
ros2 interface show lab1_interfaces/srv/SetSpeedLimits

# R3 goToPose
ros2 action send_goal --feedback /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 3.0, yaw: 1.57}}"

# R7 parameters + service (use 1.0 not 1: parameters are doubles)
ros2 param list /turtle_controller
ros2 param get  /turtle_controller max_linear_speed
ros2 param set  /turtle_controller max_linear_speed 0.5
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear: 1.0, max_angular: 1.5}"
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear: -1.0, max_angular: 1.5}"   # refused

# R4 reset
ros2 service call /reset_turtle std_srvs/srv/Trigger

# R5 square (reset + exactly 4 go_to_pose goals)
ros2 action send_goal --feedback /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}"
```

## Design patterns (bonus)
* **Strategy** – `controllers.py`: `MotionController` (abstract) / `ProportionalController`. The action server only calls
  `drive_command` / `rotate_command`, so a PID or other law can replace it without touching the server (open/closed principle).
* **Observer** – ROS 2 pub/sub itself: the controller subscribes to `/turtle1/pose` and reacts to each update.
* **Facade** – `reset_turtle` hides 4 turtlesim calls (pen off, teleport, clear, pen on) behind one `Trigger` call.

## Bag file (bonus)
```bash
ros2 launch lab1_bringup system.launch.py
ros2 bag record -o square_bag /turtle1/cmd_vel          # terminal 2
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}"
# Ctrl+C the recording, then:
ros2 bag info square_bag
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag play square_bag                                 # turtle repeats the square
```
