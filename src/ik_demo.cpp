#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ik_arm/action/compute_ik.hpp"

/**
 * @brief Demo node that sends a sequence of target poses to the IK action
 *        server, demonstrating the arm's ability to reach different poses.
 *
 * Sends 3 goals:
 *   1. A pose in front of the arm (z forward)
 *   2. A pose above and to the right
 *   3. Back to the home position
 *
 * Each goal waits for completion before sending the next.
 */
class IKDemo : public rclcpp::Node
{
public:
  using ComputeIK = ik_arm::action::ComputeIK;
  using GoalHandleIK = rclcpp_action::ClientGoalHandle<ComputeIK>;

  IKDemo()
  : Node("ik_demo")
  {
    client_ = rclcpp_action::create_client<ComputeIK>(this, "compute_ik");
  }

  void run()
  {
    using namespace std::chrono_literals;

    // Wait for the action server
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after 10s");
      return;
    }

    // ── Define target poses ──────────────────────────────────

    // Pose 1: Reach forward (0.5, 0.0, 0.3)
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.5;
    pose1.position.y = 0.0;
    pose1.position.z = 0.3;
    pose1.orientation.w = 1.0;  // Identity orientation

    // Pose 2: Reach up and right (0.3, 0.4, 0.6)
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.3;
    pose2.position.y = 0.4;
    pose2.position.z = 0.6;
    pose2.orientation.w = 1.0;

    // Pose 3: Back to home (0.3, 0.0, 0.45)
    geometry_msgs::msg::Pose pose3;
    pose3.position.x = 0.3;
    pose3.position.y = 0.0;
    pose3.position.z = 0.45;
    pose3.orientation.w = 1.0;

    // ── Send goals sequentially ──────────────────────────────

    sendGoal(pose1, "reach forward (0.5, 0.0, 0.3)");
    std::this_thread::sleep_for(2s);

    sendGoal(pose2, "reach up-right (0.3, 0.4, 0.6)");
    std::this_thread::sleep_for(2s);

    sendGoal(pose3, "return home (0.3, 0.0, 0.45)");
    std::this_thread::sleep_for(1s);

    RCLCPP_INFO(this->get_logger(), "Demo complete. Shutting down.");
    rclcpp::shutdown();
  }

private:
  rclcpp_action::Client<ComputeIK>::SharedPtr client_;

  void sendGoal(const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    RCLCPP_INFO(this->get_logger(), "Sending goal: %s", label.c_str());

    auto goal = ComputeIK::Goal();
    goal.target_pose = pose;

    auto send_goal_options = rclcpp_action::Client<ComputeIK>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, label](const GoalHandleIK::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal '%s' was rejected", label.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal '%s' accepted", label.c_str());
        }
      };

    send_goal_options.feedback_callback =
      [this, label](
      const GoalHandleIK::SharedPtr &,
      const std::shared_ptr<const ComputeIK::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(),
          "  [%s] iter=%d, pos_err=%.4f, orient_err=%.4f",
          label.c_str(),
          feedback->iteration,
          feedback->current_position_error,
          feedback->current_orientation_error);
      };

    send_goal_options.result_callback =
      [this, label](const GoalHandleIK::WrappedResult & wrapped) {
        switch (wrapped.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
              "✓ '%s' succeeded — pos_err=%.4f, orient_err=%.4f",
              label.c_str(),
              wrapped.result->position_error,
              wrapped.result->orientation_error);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(),
              "✗ '%s' aborted — pos_err=%.4f, orient_err=%.4f",
              label.c_str(),
              wrapped.result->position_error,
              wrapped.result->orientation_error);
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "'%s' finished with code %d",
              label.c_str(), static_cast<int>(wrapped.code));
            break;
        }
      };

    client_->async_send_goal(goal, send_goal_options);

    // Wait briefly for the goal to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKDemo>();
  // Run the demo sequence in a background thread so spinning works
  std::thread([node]() {node->run();}).detach();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
