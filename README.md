# IK Arm — ROS2 Inverse Kinematics Node

**Robot Programming project — Sapienza 2025/26**

6-DOF robotic arm simulation in ROS2 Jazzy.  
Implements inverse kinematics using the damped pseudo-inverse (SVD) of a numerical Jacobian, with automatic random restarts for robustness.

### Project requirements

- ✅ **ROS node** that simulates a robotic arm
- ✅ **Joint position control** via `/joint_commands` topic
- ✅ **Inverse kinematics** using pseudo-inverse on a **numerical Jacobian**
- ✅ **DH parameters** loaded from a **configuration file** (`config/arm_config.yaml`)

### Project structure

- `robot_model` — pure C++/Eigen library (FK, Jacobian, IK solver), no ROS dependencies
- `ik_arm_node` — ROS2 node that uses `robot_model` to serve IK and joint control via topics

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ik_arm ik_arm_node --ros-args --params-file \
  ~/ros2_ws/src/ik_arm/config/arm_config.yaml
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/target_pose` | `PoseStamped` | Subscribe | IK target pose — triggers IK solver |
| `/joint_commands` | `Float64MultiArray` | Subscribe | Direct joint position control |
| `/joint_states` | `JointState` | Publish | Current joint angles (100 Hz) |
| `/ee_pose` | `PoseStamped` | Publish | Current end-effector pose |

## Usage examples

**Send an IK target:**
```bash
ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**Control joints directly:**
```bash
ros2 topic pub --once /joint_commands std_msgs/Float64MultiArray \
  "{data: [0.5, -0.3, 0.0, 0.0, 0.0, 0.0]}"
```

**Monitor joint state:**
```bash
ros2 topic echo /joint_states
```

## Architecture

```
ik_arm/
├── src/
│   ├── robot_model.hpp       # DHJoint struct, RobotModel class
│   ├── robot_model.cpp     # FK, numerical Jacobian, SVD damped IK
│   └── ik_arm_node.cpp     # ROS2 node — topics, IK trigger, control loop
├── config/
│   └── arm_config.yaml     # DH parameters, IK tuning, control rate
├── CMakeLists.txt
└── package.xml
```

## Kinematics

- **DH convention**: Craig (standard), `Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)`
- **Jacobian**: 6×n numerical (finite difference, ε = 1e⁻⁸)
- **IK solver**: SVD damped pseudo-inverse with Levenberg-Marquardt damping and 5 random retries
- **6-DOF PUMA-like arm**: d₁ = 0.30 m, a₂ = 0.30 m, a₃ = 0.10 m, d₄ = 0.30 m, d₆ = 0.15 m

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ik_arm
source install/setup.bash
```
