# Autonomous Car Project
CS424 — Robotics and Control

## Team Members
| Name | Roll No | Role |
|------|---------|------|
| Mariyam Fatima | BSAI23013 | Safety Monitor |
| Sahaab Mansha | BSCS23012 | RRT* Global Planner |
| Sadia Shafeeq | BSAI23009 | Pure Pursuit Controller |


---

## Setup (First Time Only)

### 1. Install dependencies
```bash
sudo apt update
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-simulations
sudo apt install -y ros-jazzy-ros-gz-bridge ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs
pip3 install numpy scipy
```

### 2. Clone the repo
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/I-a-coder/Autonomous-car
```

### 3. Download Gazebo models
```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp Post"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Mailbox"
```

### 4. Fix world file paths (replace YOUR_USERNAME with your Linux username)
```bash
sed -i 's|/home/vboxuser/|/home/YOUR_USERNAME/|g' ~/ros2_ws/src/Autonomous-car/src/robotics_project/worlds/city.world
```

### 5. Build the workspace
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

Open **6 terminals**. Run the source command at the top of each:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

### Terminal 1 — Gazebo World
```bash
source /opt/ros/jazzy/setup.bash
gz sim ~/ros2_ws/src/Autonomous-car/src/robotics_project/worlds/city.world
```
Wait for Gazebo to fully open before continuing.

### Terminal 2 — RRT* Planner
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run robotics_project rrt_planner
```

### Terminal 3 — Pure Pursuit Controller
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run robotics_project pure_pursuit_controller
```

### Terminal 4 — ROS-Gazebo Bridge
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

### Terminal 5 — Spawn the Car
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_sim create -name prius \
  -file ~/ros2_ws/src/Autonomous-car/src/robotics_project/urdf/prius_car.sdf \
  -x 0.0 -y 0.0 -z 0.5 -Y 1.5707
```

### Terminal 6 — Safety Monitor (Member 3)
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run robotics_project safety_monitor
```

---

## How It Works

1. **RRT* Planner** computes an optimal path from (0,0) to (20,0) and publishes it to `/planned_path`
2. **Pure Pursuit Controller** makes the car follow the path smoothly
3. **Safety Monitor** watches LiDAR data on `/scan` for obstacles within 0.5m
   - If obstacle detected → stops the car via `/replan_trigger`
   - Calls `/request_replan` service → RRT* replans a new path
   - Car resumes on the new path

---

## ROS Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | nav_msgs/Path | RRT* computed path |
| `/cmd_vel` | geometry_msgs/TwistStamped | Velocity commands to car |
| `/odom` | nav_msgs/Odometry | Car position from Gazebo |
| `/scan` | sensor_msgs/LaserScan | LiDAR obstacle data |
| `/replan_trigger` | std_msgs/Bool | Safety monitor obstacle flag |
| `/safety_status` | std_msgs/String | Safety monitor status messages |
| `/cross_track_error` | std_msgs/Float32 | How far car is from path |

---

## Key Parameters to Tune
| File | Parameter | Current Value | Effect |
|------|-----------|---------------|--------|
| pure_pursuit.py | max_speed | 5.0 | Higher = faster car |
| pure_pursuit.py | lookahead_distance | 5.0 | Higher = smoother turns |
| pure_pursuit.py | max_angular_vel | 0.3 | Lower = gentler turns |
| lidar_node.py | danger_distance | 0.5m | Obstacle trigger distance |
| lidar_node.py | warn_distance | 1.5m | Warning distance |
| rrt_planner.py | goal_pos | [20.0, 0.0] | Change destination |

---

## Troubleshooting
**Car not moving?**
```bash
ros2 topic list   # should show /planned_path, /cmd_vel, /odom
```

**Build errors?**
```bash
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
```

**Gazebo crashes on launch?**
```bash
pkill -9 -f gz
pkill -9 -f gazebo
# then rerun Terminal 1
```

**Pull latest changes from GitHub?**
```bash
cd ~/ros2_ws/src/Autonomous-car
git pull
cd ~/ros2_ws && colcon build && source install/setup.bash
```

---
