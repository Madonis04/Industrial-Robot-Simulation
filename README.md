# ROS2 Industrial Robot Simulation

A comprehensive digital twin of an industrial robotic workcell built with ROS2 Humble, featuring a 6-DOF robotic arm capable of autonomous pick-and-place operations. This project demonstrates industry-standard robotics practices including hardware abstraction, motion planning, and physics-based simulation.

![Project Status](https://img.shields.io/badge/status-active-success.svg)
![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

## 🎯 Project Overview

This project implements a complete robotic simulation pipeline that mirrors real-world industrial applications. It's designed to showcase proficiency in:

- **Robot Modeling**: URDF/XACRO-based parametric robot description
- **Physics Simulation**: Gazebo Classic integration with accurate dynamics
- **Control Systems**: `ros2_control` hardware abstraction layer
- **Motion Planning**: MoveIt 2 for collision-free trajectory generation
- **Hybrid Programming**: C++ for performance-critical components, Python for orchestration

### ✨ Key Features

- ✅ **6-DOF Robotic Arm** with realistic kinematics and dynamics
- ✅ **Digital Twin Architecture** - validate operations before deploying to hardware
- ✅ **Hardware Abstraction** - swap between simulation and real robots seamlessly
- ✅ **Interactive Planning** - RViz-based motion planning interface
- ✅ **Modular Design** - separate packages for description, control, and planning
- ✅ **Industry Standards** - follows ROS2 and MoveIt 2 best practices

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      MoveIt 2 (Brain)                       │
│         Collision-Free Trajectory Generation (C++)          │
└───────────────────────┬─────────────────────────────────────┘
                        │ Joint Trajectory
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              ros2_control (Nervous System)                  │
│           Joint Trajectory Controller + YAML Config         │
└───────────────────────┬─────────────────────────────────────┘
                        │ Motor Commands
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                  Gazebo Classic (Muscles)                   │
│              Physics Engine + Visual Rendering              │
└───────────────────────┬─────────────────────────────────────┘
                        │ Joint States (Feedback)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                   RViz 2 (Visualization)                    │
│              Interactive Motion Planning UI                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 📋 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **RAM**: Minimum 8GB (16GB recommended)
- **GPU**: Integrated graphics sufficient (dedicated GPU recommended for smoother visualization)

### Required Packages

```bash
# ROS2 Humble Base
sudo apt update
sudo apt install ros-humble-desktop

# Control Stack
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control

# MoveIt 2
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-planners-ompl \
                 ros-humble-ompl

# Additional Tools
sudo apt install python3-colcon-common-extensions \
                 ros-humble-xacro
```

---

## 🚀 Installation

### 1. Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone or Create Package Structure

```bash
# Robot Description Package
ros2 pkg create --build-type ament_cmake my_robot_description
cd my_robot_description
mkdir -p urdf launch rviz config
```

### 3. Project File Structure

```
ros2_ws/
├── src/
│   ├── my_robot_description/          # Robot URDF and configuration
│   │   ├── urdf/
│   │   │   ├── my_robot.urdf.xacro   # Main robot description
│   │   │   ├── materials.xacro       # Color definitions
│   │   │   └── inertial_macros.xacro # Physics calculations
│   │   ├── launch/
│   │   │   ├── display.launch.py     # RViz visualization
│   │   │   └── gazebo.launch.py      # Gazebo simulation
│   │   └── config/
│   │       └── my_controllers.yaml   # ros2_control config
│   │
│   └── my_robot_moveit_config/        # MoveIt 2 configuration
│       ├── config/
│       │   ├── moveit_controllers.yaml
│       │   ├── joint_limits.yaml
│       │   └── *.srdf
│       └── launch/
│           └── moveit_gazebo.launch.py
```

---

## 🔧 Configuration Files

### Key Configuration: `my_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

### MoveIt Controller Integration: `moveit_controllers.yaml`

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
```

---

## 🎮 Usage

### Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Launch Options

#### Option 1: RViz Only (Model Visualization)
```bash
ros2 launch my_robot_description display.launch.py
```
- **Use Case**: Verify robot model and kinematics
- **Features**: Joint sliders, TF visualization

#### Option 2: Gazebo Simulation (Physics)
```bash
ros2 launch my_robot_description gazebo.launch.py
```
- **Use Case**: Test physics and controllers
- **Features**: Gravity, collision detection, motor control

#### Option 3: Full MoveIt Integration (Recommended)
```bash
ros2 launch my_robot_moveit_config moveit_gazebo.launch.py
```
- **Use Case**: Complete motion planning and execution
- **Features**: All of the above + path planning, collision avoidance

---

## 🎯 Interactive Motion Planning

### Step-by-Step Guide

1. **Launch the Full System**
   ```bash
   ros2 launch my_robot_moveit_config moveit_gazebo.launch.py
   ```

2. **Verify Components**
   - **Gazebo**: Robot spawns and stands upright
   - **RViz**: Planning interface loads with interactive markers
   - **Terminal**: No red errors, controllers report "active"

3. **Plan a Motion**
   - In RViz, locate the **interactive marker** (colored ball/arrows) at the robot's end effector
   - Drag the marker to a new position
   - Click **"Plan"** in the MotionPlanning panel
   - Observe the ghost robot showing the planned path

4. **Execute Motion**
   - Click **"Execute"** button
   - Watch the robot in Gazebo move to match the planned trajectory

### Common RViz Settings

| Setting | Recommended Value | Purpose |
|---------|------------------|---------|
| Fixed Frame | `base_link` | Robot coordinate reference |
| Planning Group | `arm` | Joint group to control |
| Query Goal State | ✅ Checked | Show interactive marker |
| Show Robot Visual | ✅ Checked | Display robot mesh |

---


## 📚 Technical Deep Dive

### Robot Model Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Degrees of Freedom | 6 | Full 3D manipulation |
| Base Dimensions | 0.4m × 0.4m × 0.2m | Stable mounting |
| Link Length | 0.6m (primary) | Human-scale reach |
| Total Mass | ~20kg | Realistic industrial weight |
| Joint Limits | ±3.14 rad | Full rotation capability |

### Coordinate Frames Hierarchy

```
world (fixed)
  └── base_link (robot anchor)
      └── link_1 (rotates Z-axis)
          └── link_2 (rotates Y-axis)
              └── link_3 (rotates Y-axis)
                  └── link_4 (rotates Y-axis)
                      └── link_5 (rotates Z-axis)
                          └── link_6 (rotates X-axis)
```

### Control Loop Architecture

1. **MoveIt** computes collision-free path → outputs waypoints
2. **JointTrajectoryController** interpolates between waypoints → sends position commands
3. **GazeboSystem** (hardware interface) → applies commands to simulated joints
4. **Joint encoders** → publish feedback to `/joint_states`
5. **Loop repeats** at 100Hz (configurable in YAML)

---

## 🎓 Learning Outcomes

By completing this project, you demonstrate:

### Technical Skills
- ✅ URDF/XACRO parametric modeling
- ✅ Gazebo physics plugin integration
- ✅ ros2_control hardware abstraction
- ✅ MoveIt 2 motion planning pipeline
- ✅ ROS2 launch file orchestration
- ✅ C++/Python hybrid architecture

### Software Engineering
- ✅ Modular package design
- ✅ Configuration management (YAML)
- ✅ Debugging distributed systems
- ✅ Version control best practices
- ✅ Documentation standards
- 
---

## 🔮 Future Enhancements

### Planned Features
- [ ] Custom gripper integration with vacuum/magnetic attach
- [ ] Visual servoing for camera-based manipulation
- [ ] Obstacle avoidance using Octomap
- [ ] Multi-robot coordination
- [ ] Hardware deployment guide (UR5/Fanuc)

### Advanced Modifications
- [ ] Replace geometric primitives with CAD meshes
- [ ] Add force/torque sensors simulation
- [ ] Implement custom IK solver in C++
- [ ] Create ROS2 actions for pick-and-place sequences

---

## 📖 Resources & References

### Official Documentation
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/main/index.html)
- [Gazebo Classic](http://classic.gazebosim.org/)
- [ros2_control](https://control.ros.org/)

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
- [GitHub Discussions](https://github.com/ros-planning/moveit2/discussions)

---

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Mayank Ranade**  
📧 mayankvranade@gmail.com.com  
🔗 [LinkedIn](https://linkedin.com/in/mayankranade) | [GitHub](https://github.com/Madonis04)

---

## 🙏 Acknowledgments

- **ROS2 Community** for comprehensive documentation
- **MoveIt Maintainers** for the powerful planning framework
- **Techolution** interview preparation inspiration
- **Open Robotics** for Gazebo simulator

---

## 📊 Project Statistics

- **Lines of Code**: ~1,500 (URDF/YAML/Python/Launch)
- **Configuration Files**: 8
- **Packages**: 2
- **Controllers**: 2 (broadcaster + trajectory)
- **Planning Algorithms**: 3 (RRT, CHOMP, Pilz)

---

**⭐ If this project helped you, please consider giving it a star!**

*Last Updated: December 2025*
