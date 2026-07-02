# ik_arm — Inverse Kinematics ROS2 Node

A ROS2 node that simulates a 6-DOF robotic arm and provides inverse
kinematics using the damped pseudo-inverse of a numerical Jacobian
(SVD-based Levenberg–Marquardt).

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  IKArmNode                       │
│                                                   │
│  /target_pose  (sub) ───┐                        │
│  /joint_commands (sub) ──┤                        │
│                          │                        │
│                   ┌──────▼──────┐                │
│                   │  RobotModel │                │
│                   │  (static    │                │
│                   │   library)  │                │
│                   │  FK, Jac,   │                │
│                   │  IK (SVD)   │                │
│                   └──────▲──────┘                │
│                          │                        │
│  /compute_ik   (action) ─┘                        │
│                                                   │
│  /joint_states (pub)                              │
│  /ee_pose      (pub)                              │
└─────────────────────────────────────────────────┘
```

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select ik_arm
source install/setup.bash

# Run the node
ros2 run ik_arm ik_arm_node

# In another terminal, run the demo
ros2 run ik_arm ik_demo
```

Or via the launch file:

```bash
ros2 launch ik_arm ik_arm.launch.py joint_move_speed:=0.05
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/target_pose` | `PoseStamped` | Sub | IK target pose for the end-effector |
| `/joint_commands` | `Float64MultiArray` | Sub | Direct joint angle command |
| `/joint_states` | `JointState` | Pub | Current joint positions |
| `/ee_pose` | `PoseStamped` | Pub | Current end-effector pose |

## Action

| Action | Type | Description |
|--------|------|-------------|
| `/compute_ik` | `ComputeIK` | Compute IK for a target pose and move the arm |

**Goal:** `geometry_msgs/Pose target_pose`

**Result:** `float64[] joint_positions`, `bool success`,
`float64 position_error`, `float64 orientation_error`

**Feedback:** `float64 current_position_error`,
`float64 current_orientation_error`, `int32 iteration`

## Robot Model (PUMA 560-like)

| Joint | a (m) | d (m) | α (rad) | θ₀ff (rad) | Range (rad) |
|-------|-------|-------|---------|-----------|-------------|
| 1 | 0.00 | 0.30 | +π/2 | 0.0 | ±π |
| 2 | 0.30 | 0.00 | 0 | 0.0 | ±2.094 |
| 3 | 0.10 | 0.00 | −π/2 | 0.0 | ±2.094 |
| 4 | 0.00 | 0.30 | +π/2 | 0.0 | ±π |
| 5 | 0.00 | 0.00 | −π/2 | 0.0 | ±2.094 |
| 6 | 0.00 | 0.15 | 0 | 0.0 | ±π |

All joints are revolute. DH convention: Craig.

## Parameters

All parameters can be set via YAML config file or the launch file.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `joint_move_speed` | 0.08 | Max joint speed (rad/s) |
| `control_rate` | 100.0 | Control loop rate (Hz) |
| `position_tolerance` | 0.001 | IK position tolerance (m) |
| `orientation_tolerance` | 0.01 | IK orientation tolerance (rad) |
| `ik_damping` | 0.1 | SVD damping factor λ |
| `ik_max_iterations` | 200 | Max iterations per attempt |

## Implementation Details

- **Forward kinematics**: Craig DH convention, 4×4 homogeneous transforms
- **Jacobian**: Numerical (finite difference, ε = 1e⁻⁸)
- **IK solver**: SVD damped pseudo-inverse (Levenberg–Marquardt)
  `Δq = Σᵢ σᵢ/(σᵢ²+λ²)·(Uᵢ·e)·Vᵢ`
- **Retries**: Up to 5 random restarts if no convergence
- **Control loop**: 100 Hz smooth interpolation with deadband (1e⁻⁶)
- **Action**: Runs IK in a worker thread with iteration feedback

## Tests

```bash
colcon test --packages-select ik_arm
colcon test-result --verbose
```

- `test_robot_model`: FK, Jacobian dimensions, IK convergence, clamping
- `test_ik_arm_node`: Node creation, parameters, DH sizes

## License

MIT
