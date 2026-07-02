// Copyright (c) 2026 Zeyad Kandil

#ifndef IK_ARM__IK_ARM_NODE_HPP_
#define IK_ARM__IK_ARM_NODE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ik_arm/action/compute_ik.hpp"
#include "ik_arm/robot_model.hpp"

namespace ik_arm
{

/**
 * @brief ROS2 node that provides inverse kinematics for a robotic arm.
 *
 * Subscribes to:
 *   /target_pose      (PoseStamped)  — IK target pose
 *   /joint_commands   (Float64MultiArray) — direct joint angle command
 *
 * Publishes:
 *   /joint_states     (JointState)
 *   /ee_pose          (PoseStamped)  — current end-effector pose
 *
 * Action server:
 *   /compute_ik       (ComputeIK)    — compute IK and move arm to target
 *
 * Runs a 100 Hz control loop that smoothly interpolates toward the
 * target joint configuration at a configurable joint_move_speed.
 */
class IKArmNode : public rclcpp::Node
{
public:
  explicit IKArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Action types ───────────────────────────────────────
  using ComputeIK = ik_arm::action::ComputeIK;
  using GoalHandleIK = rclcpp_action::ServerGoalHandle<ComputeIK>;

  // ── Callbacks ──────────────────────────────────────────
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onJointCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void controlLoop();

  // ── Action server callbacks ────────────────────────────
  rclcpp_action::GoalResponse handleIKGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputeIK::Goal> goal);
  rclcpp_action::CancelResponse handleIKCancel(
    std::shared_ptr<GoalHandleIK> goal_handle);
  void handleIKAccepted(std::shared_ptr<GoalHandleIK> goal_handle);

  /// Execute IK computation (runs in a worker thread).
  void executeIK(std::shared_ptr<GoalHandleIK> goal_handle);

  // ── Parameter helpers ──────────────────────────────────
  void declareParameters();
  void buildRobotModel();
  void buildDefaultModel();

  // ── Members ────────────────────────────────────────────
  RobotModel model_;
  Eigen::VectorXd current_joints_;
  Eigen::VectorXd target_joints_;
  std::vector<std::string> joint_names_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp_action::Server<ComputeIK>::SharedPtr ik_action_server_;

  // Configurable parameters
  double control_rate_;
  double move_speed_;
  double pos_tol_;
  double orient_tol_;
  double ik_damping_;
  int ik_max_iters_;
};

}  // namespace ik_arm

#endif  // IK_ARM__IK_ARM_NODE_HPP_
