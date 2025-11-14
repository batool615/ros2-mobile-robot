# My Robot Simulation (ROS2 + Gazebo + RViz)

## 🚀 Overview
This repository contains a complete **ROS2 mobile robot simulation** built using:

- **URDF + XACRO** for robot modeling  
- **Gazebo** for physics and sensor simulation  
- **RViz** for visualization  
- Python nodes for **differential drive**, **wall following**, and **smooth motion**  

The robot includes:
- Differential drive wheels  
- Caster wheel  
- Front RGB camera  
- Hokuyo 2D LiDAR  
- Multiple simulation worlds  

This project was created as part of the **ROS Course Project**.

---

## 📂 Project Structure

```
my_robot/
├── launch/
│   ├── my_robot_gazebo.launch.py
│   ├── my_robot_rviz.launch.py
│   ├── maze_sim.launch.py
│   ├── world_sim.launch.py
│   ├── world_sim_manual.launch.py
│
├── urdf/
│   ├── myrobot.xacro
│   ├── robot.gazebo
│
├── worlds/
│   ├── maze.world
│   ├── indoor.world
│   ├── outdoor.world
│   ├── obstacles.world
│   ├── maze_big.world
│
├── meshes/
│   ├── hokuyo.dae
│
├── rviz/
│   ├── robot_view.rviz
│
├── my_robot/
│   ├── diff_drive.py
│   ├── wall_follow_left.py
│   ├── diff_drive_smooth.py
│
├── package.xml
├── setup.py
├── setup.cfg
```

---

## 🤖 Robot Description

### 🟩 Chassis  
URDF modeled chassis with proper inertia and collision.

### 🟠 Differential Drive  
- Joints: `left_wheel_hinge`, `right_wheel_hinge`
- Wheel separation: **0.3 m**
- Wheel diameter: **0.2 m**

### 🔵 Camera Sensor  
- Topic: `/my_robot/camera1/image_raw`
- Update rate: 30 Hz  

### 🔴 Hokuyo LiDAR  
- Topic: `/scan`
- 360° horizontal scan  

---

## 🏁 How to Run the Simulation

### 1️⃣ Build the workspace
```bash
colcon build
source install/setup.bash
```

### 2️⃣ Launch Gazebo
```bash
ros2 launch my_robot my_robot_gazebo.launch.py
```

### 3️⃣ Launch RViz
```bash
ros2 launch my_robot my_robot_rviz.launch.py
```

---

## 🎮 Teleoperation (Optional)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🧠 Autonomous Behaviors
```bash
ros2 run my_robot wall_follow_left
ros2 run my_robot diff_drive_smooth
```

---

## 👩‍💻 Author
**Batool Aloran — AI & Data Science Student**  
The Hashemite University — ROS Course Project
