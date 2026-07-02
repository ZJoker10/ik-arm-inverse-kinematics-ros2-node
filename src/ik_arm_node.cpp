#include "ik_arm/ik_arm_node.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace ik_arm
{

IKArmNode::IKArmNode(const rclcpp::NodeOptions & options)
: Node("ik_arm_node", options)
{
  declareParameters();
  buildRobotModel();

  // Publishers
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
  ee_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "ee_pose", 10);

  // Subscribers
  target_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10,
      std::bind(&IKArmNode::onTargetPose, this, std::placeholders::_1));
  joint_cmd_sub_ =
    this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_commands", 10,
      std::bind(&IKArmNode::onJointCommand, this, std::placeholders::_1));

  // Action server: /compute_ik
  ik_action_server_ = rclcpp_action::create_server<ComputeIK>(
    this,
    "compute_ik",
    std::bind(&IKArmNode::handleIKGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&IKArmNode::handleIKCancel, this, std::placeholders::_1),
    std::bind(&IKArmNode::handleIKAccepted, this, std::placeholders::_1));

  // Initialize state (straight-up home pose: all zeros)
  current_joints_ = Eigen::VectorXd::Zero(model_.numJoints());
  target_joints_ = current_joints_;

  // Control timer
  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&IKArmNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "IK Arm Node started — %zu DOF, rate = %.0f Hz, speed = %.2f rad/s",
    model_.numJoints(), control_rate_, move_speed_);
  RCLCPP_INFO(this->get_logger(),
    "  Sub: /target_pose, /joint_commands");
  RCLCPP_INFO(this->get_logger(),
    "  Pub: /joint_states, /ee_pose");
  RCLCPP_INFO(this->get_logger(),
    "  Action: /compute_ik");
}

// ═══════════════════════════════════════════════════════════════
//  Parameter declaration
// ═══════════════════════════════════════════════════════════════

void IKArmNode::declareParameters()
{
  this->declare_parameter("robot_name", "ik_arm");
  this->declare_parameter("joint_names",
    std::vector<std::string>{
      "joint1", "joint2", "joint3",
      "joint4", "joint5", "joint6"});

  const std::vector<double> dfl_a = {0.0, 0.30, 0.10, 0.0, 0.0, 0.0};
  const std::vector<double> dfl_d = {0.30, 0.0, 0.0, 0.30, 0.0, 0.15};
  const std::vector<double> dfl_alp = {1.5708, 0.0, -1.5708,
    1.5708, -1.5708, 0.0};
  const std::vector<double> dfl_to = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> dfl_min = {-3.1416, -2.0944, -2.0944,
    -3.1416, -2.0944, -3.1416};
  const std::vector<double> dfl_max = {3.1416, 2.0944, 2.0944,
    3.1416, 2.0944, 3.1416};

  this->declare_parameter("joint_a", dfl_a);
  this->declare_parameter("joint_d", dfl_d);
  this->declare_parameter("joint_alpha", dfl_alp);
  this->declare_parameter("joint_theta_offset", dfl_to);
  this->declare_parameter("joint_min", dfl_min);
  this->declare_parameter("joint_max", dfl_max);

  this->declare_parameter("position_tolerance", 0.001);
  this->declare_parameter("orientation_tolerance", 0.01);
  this->declare_parameter("ik_damping", 0.1);
  this->declare_parameter("ik_max_iterations", 200);

  this->declare_parameter("control_rate", 100.0);
  this->declare_parameter("joint_move_speed", 0.08);
}

void IKArmNode::buildRobotModel()
{
  const auto names = this->get_parameter("joint_names").as_string_array();
  const auto a_vals = this->get_parameter("joint_a").as_double_array();
  const auto d_vals = this->get_parameter("joint_d").as_double_array();
  const auto alpha_vals = this->get_parameter("joint_alpha").as_double_array();
  const auto theta_off = this->get_parameter("joint_theta_offset").as_double_array();
  const auto min_vals = this->get_parameter("joint_min").as_double_array();
  const auto max_vals = this->get_parameter("joint_max").as_double_array();

  control_rate_ = this->get_parameter("control_rate").as_double();
  move_speed_ = this->get_parameter("joint_move_speed").as_double();
  pos_tol_ = this->get_parameter("position_tolerance").as_double();
  orient_tol_ = this->get_parameter("orientation_tolerance").as_double();
  ik_damping_ = this->get_parameter("ik_damping").as_double();
  ik_max_iters_ = this->get_parameter("ik_max_iterations").as_int();

  const size_t n = names.size();
  if (a_vals.size() != n || d_vals.size() != n ||
    alpha_vals.size() != n || theta_off.size() != n ||
    min_vals.size() != n || max_vals.size() != n)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch; using defaults.");
    buildDefaultModel();
    return;
  }

  std::vector<DHJoint> joints(n);
  joint_names_ = names;
  for (size_t i = 0; i < n; ++i) {
    joints[i].name = names[i];
    joints[i].a = a_vals[i];
    joints[i].d = d_vals[i];
    joints[i].alpha = alpha_vals[i];
    joints[i].theta_offset = theta_off[i];
    joints[i].position_min = min_vals[i];
    joints[i].position_max = max_vals[i];
    joints[i].joint_type = "revolute";
  }
  model_.setJoints(joints);
  RCLCPP_INFO(this->get_logger(), "Loaded %zu-DOF arm from parameters", n);
}

void IKArmNode::buildDefaultModel()
{
  constexpr double kPi = 3.141592653589793;
  std::vector<DHJoint> joints(6);
  joint_names_ = {"joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6"};
  // clang-format off
  joints[0] = {"joint1", 0.0, 0.30, kPi / 2, 0.0, -kPi, kPi, "revolute"};
  joints[1] = {"joint2", 0.30, 0.0, 0.0, 0.0, -2.0944, 2.0944, "revolute"};
  joints[2] = {"joint3", 0.10, 0.0, -kPi / 2, 0.0, -2.0944, 2.0944, "revolute"};
  joints[3] = {"joint4", 0.0, 0.30, kPi / 2, 0.0, -kPi, kPi, "revolute"};
  joints[4] = {"joint5", 0.0, 0.0, -kPi / 2, 0.0, -2.0944, 2.0944, "revolute"};
  joints[5] = {"joint6", 0.0, 0.15, 0.0, 0.0, -kPi, kPi, "revolute"};
  // clang-format on
  model_.setJoints(joints);
  RCLCPP_WARN(this->get_logger(), "Using default 6-DOF arm model");
}

// ═══════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════

void IKArmNode::onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;

  Eigen::Matrix4d T_desired = Eigen::Matrix4d::Identity();
  T_desired(0, 3) = p.x;
  T_desired(1, 3) = p.y;
  T_desired(2, 3) = p.z;

  // Quaternion → rotation matrix
  const double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
  // clang-format off
  T_desired.block<3, 3>(0, 0) <<
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
    2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
    2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy);
  // clang-format on

  Eigen::VectorXd result;
  if (model_.solveIK(current_joints_, T_desired, result,
      ik_max_iters_, pos_tol_, orient_tol_, ik_damping_))
  {
    target_joints_ = result;
    RCLCPP_INFO(this->get_logger(),
      "IK solved — target (%.2f, %.2f, %.2f) → moving arm",
      p.x, p.y, p.z);
  } else {
    RCLCPP_WARN(this->get_logger(),
      "IK did NOT converge for (%.2f, %.2f, %.2f)",
      p.x, p.y, p.z);
  }
}

void IKArmNode::onJointCommand(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (static_cast<size_t>(msg->data.size()) != model_.numJoints()) {
    RCLCPP_WARN(this->get_logger(),
      "Expected %zu joint values, got %zu",
      model_.numJoints(), msg->data.size());
    return;
  }
  for (size_t i = 0; i < model_.numJoints(); ++i) {
    target_joints_(i) = msg->data[i];
  }
  target_joints_ = model_.clampJoints(target_joints_);
  RCLCPP_INFO(this->get_logger(), "Direct joint command received");
}

// ═══════════════════════════════════════════════════════════════
//  Action server
// ═══════════════════════════════════════════════════════════════

rclcpp_action::GoalResponse IKArmNode::handleIKGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ComputeIK::Goal>/*goal*/)
{
  RCLCPP_INFO(this->get_logger(), "Received IK action goal");
  // Always accept
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IKArmNode::handleIKCancel(
  std::shared_ptr<GoalHandleIK>/*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "IK action goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void IKArmNode::handleIKAccepted(std::shared_ptr<GoalHandleIK> goal_handle)
{
  // Run IK in a worker thread so the action doesn't block the spinner
  std::thread{&IKArmNode::executeIK, this, std::move(goal_handle)}.detach();
}

void IKArmNode::executeIK(std::shared_ptr<GoalHandleIK> goal_handle)
{
  const auto target = goal_handle->get_goal()->target_pose;
  const auto & p = target.position;
  const auto & q = target.orientation;

  // Build target transform
  Eigen::Matrix4d T_desired = Eigen::Matrix4d::Identity();
  T_desired(0, 3) = p.x;
  T_desired(1, 3) = p.y;
  T_desired(2, 3) = p.z;
  const double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
  // clang-format off
  T_desired.block<3, 3>(0, 0) <<
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
    2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
    2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy);
  // clang-format on

  // Run IK with retries, publishing feedback
  constexpr double kDt = 0.5;
  Eigen::VectorXd joint_angles = model_.clampJoints(current_joints_);
  bool converged = false;
  double final_pos_err = 0.0, final_orient_err = 0.0;

  for (int retry = 0; retry <= 5 && !converged; ++retry) {
    if (retry > 0) {
      joint_angles = model_.randomConfiguration();
    }

    for (int iter = 0; iter < ik_max_iters_; ++iter) {
      const Eigen::Matrix4d T_cur = model_.forwardKinematics(joint_angles);
      const Eigen::Vector3d p_cur = T_cur.block<3, 1>(0, 3);
      const Eigen::Matrix3d R_cur = T_cur.block<3, 3>(0, 0);

      // Position error
      const Eigen::Vector3d pos_error = T_desired.block<3, 1>(0, 3) - p_cur;

      // Orientation error (angle-axis)
      const Eigen::Matrix3d R_err = T_desired.block<3, 3>(0, 0) * R_cur.transpose();
      const double cos_angle =
        std::clamp((R_err.trace() - 1.0) / 2.0, -1.0, 1.0);
      const double angle = std::acos(cos_angle);
      Eigen::Vector3d orient_error = Eigen::Vector3d::Zero();
      if (angle > 1e-10) {
        Eigen::Vector3d axis;
        axis << R_err(2, 1) - R_err(1, 2),
          R_err(0, 2) - R_err(2, 0),
          R_err(1, 0) - R_err(0, 1);
        const double nrm = axis.norm();
        if (nrm > 1e-10) {
          axis /= (2.0 * std::sin(angle));
          orient_error = angle * axis;
        }
      }

      const double pos_err = pos_error.norm();
      const double orient_err = orient_error.norm();
      final_pos_err = pos_err;
      final_orient_err = orient_err;

      // Publish feedback every 5 iterations
      if (iter % 5 == 0) {
        auto feedback = std::make_shared<ComputeIK::Feedback>();
        feedback->current_position_error = pos_err;
        feedback->current_orientation_error = orient_err;
        feedback->iteration = iter;
        goal_handle->publish_feedback(feedback);
      }

      // Check convergence
      if (pos_err < pos_tol_ && orient_err < orient_tol_) {
        converged = true;
        break;
      }

      // IK step via SVD damped pseudo-inverse
      const Eigen::MatrixXd J = model_.numericalJacobian(joint_angles);
      Eigen::VectorXd e(6);
      e.head<3>() = pos_error;
      e.tail<3>() = orient_error;

      Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J, Eigen::ComputeFullU | Eigen::ComputeFullV);
      const Eigen::VectorXd sv = svd.singularValues();
      Eigen::VectorXd dq = Eigen::VectorXd::Zero(J.cols());
      for (int i = 0; i < sv.size(); ++i) {
        const double s = sv(i);
        dq += (s / (s * s + ik_damping_ * ik_damping_)) *
          svd.matrixU().col(i).dot(e) * svd.matrixV().col(i);
      }

      joint_angles += kDt * dq;
      joint_angles = model_.clampJoints(joint_angles);
    }
  }

  // Command the arm to move to the solution
  target_joints_ = joint_angles;

  // Send result
  auto result = std::make_shared<ComputeIK::Result>();
  result->success = converged;
  result->position_error = final_pos_err;
  result->orientation_error = final_orient_err;
  result->joint_positions.resize(joint_angles.size());
  for (int i = 0; i < joint_angles.size(); ++i) {
    result->joint_positions[i] = joint_angles(i);
  }

  if (converged) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(),
      "IK action succeeded — pos_err=%.4f, orient_err=%.4f",
      final_pos_err, final_orient_err);
  } else {
    goal_handle->abort(result);
    RCLCPP_WARN(this->get_logger(),
      "IK action failed — pos_err=%.4f, orient_err=%.4f",
      final_pos_err, final_orient_err);
  }
}

// ═══════════════════════════════════════════════════════════════
//  Control loop
// ═══════════════════════════════════════════════════════════════

void IKArmNode::controlLoop()
{
  // Deadband — snap to target when close enough to prevent micro-wiggle
  const Eigen::VectorXd delta = target_joints_ - current_joints_;
  if (delta.cwiseAbs().maxCoeff() < 1e-6) {
    current_joints_ = target_joints_;
  } else {
    // Smooth interpolation toward target
    const double max_dq = move_speed_ / control_rate_;
    const Eigen::VectorXd step = delta.cwiseMax(-max_dq).cwiseMin(max_dq);
    current_joints_ += step;
  }

  // Forward kinematics
  const Eigen::Matrix4d T_ee = model_.forwardKinematics(current_joints_);

  // Publish joint states
  auto js = sensor_msgs::msg::JointState();
  js.header.stamp = this->now();
  js.header.frame_id = "base_link";
  js.name = joint_names_;
  js.position.resize(model_.numJoints());
  for (size_t i = 0; i < model_.numJoints(); ++i) {
    js.position[i] = current_joints_(i);
  }
  joint_state_pub_->publish(js);

  // Publish EE pose
  auto ee_msg = geometry_msgs::msg::PoseStamped();
  ee_msg.header.stamp = js.header.stamp;
  ee_msg.header.frame_id = "base_link";
  ee_msg.pose.position.x = T_ee(0, 3);
  ee_msg.pose.position.y = T_ee(1, 3);
  ee_msg.pose.position.z = T_ee(2, 3);
  Eigen::Quaterniond quat(T_ee.block<3, 3>(0, 0));
  ee_msg.pose.orientation.x = quat.x();
  ee_msg.pose.orientation.y = quat.y();
  ee_msg.pose.orientation.z = quat.z();
  ee_msg.pose.orientation.w = quat.w();
  ee_pose_pub_->publish(ee_msg);
}

}  // namespace ik_arm

// ═══════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════

#ifndef BUILD_TESTING
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ik_arm::IKArmNode>());
  rclcpp::shutdown();
  return 0;
}
#endif  // BUILD_TESTING
