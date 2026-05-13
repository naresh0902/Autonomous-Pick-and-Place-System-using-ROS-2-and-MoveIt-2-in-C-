# 🤖 Franka Panda Color Sorting Robot (C++ Edition)

A **ROS 2-based intelligent color sorting system** powered by the **Franka Emika Panda robotic arm**. This project is written in **native C++** and seamlessly integrates **OpenCV computer vision**, the **MoveIt 2 C++ API (`MoveGroupInterface`)**, and **Gazebo simulation** to create a high-performance, autonomous pick-and-place system that detects and sorts colored objects with precision.

---

## 🎥 Pick and Place Demo  

*(Insert your video link or GIF here showing the simulation running)*
`[Watch the Simulation Demo](link-to-your-video.mp4)`

---

## ✨ Features

- ⚡ **Native C++ Performance**: Fully implemented in C++ utilizing `rclcpp` for minimal latency and high-speed execution.
- 🎨 **Color Detection Pipeline**: Real-time C++ OpenCV vision node for Red, Green, and Blue object detection.
- 🦾 **Advanced Motion Planning**: Direct integration with the MoveIt 2 C++ API (`MoveGroupInterface`) for precise Inverse Kinematics (IK) and Cartesian trajectory planning.
- 🎯 **Coordinate Transformations**: Dynamic spatial reasoning using `tf2_ros` to translate pixel coordinates into the robot's base frame.
- 🔄 **Dynamic Target Selection**: Easily switch between target colors without restarting the core system.
- 📊 **Visual Feedback**: Integrated RViz visualization for motion planning and execution.

---

## 📋 Table of Contents

1. [🎮 Running the Project](#-running-the-project)
2. [📂 Project Structure](#-project-structure)
3. [🛠️ Customization Guide](#️-customization-guide)

---


## 🎮 Running the Project

The system is separated into the main simulation/planning environment and the C++ MoveIt client node. You will need to open **two terminal windows**.

### Terminal 1: Launch the System

This terminal will start the Gazebo simulation, spawn the Franka Panda arm, initialize the MoveIt 2 planning scene, and start the C++ OpenCV color detection node.

```bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```
*Wait for all nodes to initialize (you will see "Ready to plan" messages).*

### Terminal 2: Start the Pick-and-Place Client

This terminal runs our C++ MoveIt client. It loads the `robot_description`, waits for the vision node to lock onto the specified color, and then computes and executes the Cartesian trajectories to pick up the block.

```bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_moveit client.launch.py target_color:=R
```

**Switching Target Colors:**
You can change the target color dynamically by stopping the client node (`Ctrl+C`) and re-launching it with a different parameter:

```bash
# Sort Green objects
ros2 launch panda_moveit client.launch.py target_color:=G

# Sort Blue objects
ros2 launch panda_moveit client.launch.py target_color:=B
```

---

## 📂 Project Structure

This workspace is entirely native C++ and is divided into several modular ROS 2 packages:

- **`panda_description`**: Contains the URDF, meshes, and visual configurations for the Franka Panda.
- **`panda_controller`**: Contains the `slider_controller` C++ node for translating JointState commands to JointTrajectories.
- **`panda_vision`**: An `ament_cmake` package containing `color_detector.cpp`, which handles OpenCV masking and `tf2_ros` transformations.
- **`panda_moveit`**: Contains the MoveIt configuration (SRDF, kinematics) and the `pick_and_place.cpp` logic utilizing `MoveGroupInterface`.
- **`panda_bringup`**: Contains the main launch files to bring up the entire system.

---

## 🛠️ Customization Guide

### 1. Adjusting Color Thresholds
If you want to detect new objects or change lighting, you can adjust the HSV ranges in `panda_vision/src/color_detector.cpp`:
```cpp
std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_ranges = {
  {"R", {cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)}},
  {"G", {cv::Scalar(55, 200, 200), cv::Scalar(60, 255, 255)}},
  {"B", {cv::Scalar(90, 200, 200), cv::Scalar(128, 255, 255)}}
};
```

### 2. Modifying the Drop Locations
To change where the robot drops the sorted blocks, modify the hardcoded joint states in the constructor of `panda_moveit/src/pick_and_place.cpp`:
```cpp
drop_joints_ = {deg2rad(-155.0), deg2rad(30.0), deg2rad(-20.0), ...};
```

### 3. Tuning the Motion Planner
To speed up or slow down the robot's movements, you can adjust the velocity and acceleration scaling factors in `pick_and_place.cpp`:
```cpp
arm_group_->setMaxVelocityScalingFactor(0.1);      // Set between 0.0 and 1.0
arm_group_->setMaxAccelerationScalingFactor(0.1);  // Set between 0.0 and 1.0
```
