#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "ik_arm/ik_arm_node.hpp"

namespace ik_arm
{

class IKArmNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<IKArmNode>();
  }

  void TearDown() override
  {
    node.reset();
  }

  std::shared_ptr<IKArmNode> node;
};

// Verify that the node can be created without crashing.
TEST_F(IKArmNodeTest, NodeCreation)
{
  EXPECT_NE(node, nullptr);
}

// Verify that default parameters are declared with expected values.
TEST_F(IKArmNodeTest, DefaultParameters)
{
  EXPECT_DOUBLE_EQ(node->get_parameter("control_rate").as_double(), 100.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("joint_move_speed").as_double(), 0.08);
  EXPECT_DOUBLE_EQ(node->get_parameter("position_tolerance").as_double(), 0.001);
  EXPECT_DOUBLE_EQ(node->get_parameter("orientation_tolerance").as_double(), 0.01);
  EXPECT_DOUBLE_EQ(node->get_parameter("ik_damping").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("ik_max_iterations").as_int(), 200);
  EXPECT_EQ(node->get_parameter("robot_name").as_string(), "ik_arm");
}

// Verify that parameters can be overridden via NodeOptions.
TEST_F(IKArmNodeTest, NodeOptionsParameterOverrides)
{
  auto options = rclcpp::NodeOptions().parameter_overrides({
      {"control_rate", 50.0},
      {"joint_move_speed", 0.05},
      {"position_tolerance", 0.01},
  });
  auto custom_node = std::make_shared<IKArmNode>(options);

  EXPECT_DOUBLE_EQ(custom_node->get_parameter("control_rate").as_double(), 50.0);
  EXPECT_DOUBLE_EQ(custom_node->get_parameter("joint_move_speed").as_double(), 0.05);
  EXPECT_DOUBLE_EQ(custom_node->get_parameter("position_tolerance").as_double(), 0.01);

  // Non-overridden parameters should keep defaults.
  EXPECT_DOUBLE_EQ(custom_node->get_parameter("orientation_tolerance").as_double(), 0.01);
  EXPECT_DOUBLE_EQ(custom_node->get_parameter("ik_damping").as_double(), 0.1);
}

// Verify that the node has 6 joints by default (6-DOF PUMA-like arm).
TEST_F(IKArmNodeTest, SixDofByDefault)
{
  // Access the model through the parameter structure —
  // the node loads joints from default parameters on construction.
  const auto joint_names = node->get_parameter("joint_names").as_string_array();
  EXPECT_EQ(joint_names.size(), 6U);
  EXPECT_EQ(joint_names[0], "joint1");
  EXPECT_EQ(joint_names[5], "joint6");
}

// Verify that DH parameters have the expected sizes.
TEST_F(IKArmNodeTest, DHParameterSizes)
{
  const auto a_vals = node->get_parameter("joint_a").as_double_array();
  const auto d_vals = node->get_parameter("joint_d").as_double_array();
  const auto alpha_vals = node->get_parameter("joint_alpha").as_double_array();

  EXPECT_EQ(a_vals.size(), 6U);
  EXPECT_EQ(d_vals.size(), 6U);
  EXPECT_EQ(alpha_vals.size(), 6U);
}

}  // namespace ik_arm

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
