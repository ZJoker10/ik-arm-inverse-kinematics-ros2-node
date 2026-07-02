#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "ik_arm/robot_model.hpp"

namespace ik_arm
{

// Create a simple 2-DOF planar arm for deterministic testing.
//   Joint 1: a=1.0, d=0, alpha=0, theta_offset=0  (shoulder)
//   Joint 2: a=1.0, d=0, alpha=0, theta_offset=0  (elbow)
RobotModel makePlanar2DOF()
{
  std::vector<DHJoint> joints(2);
  joints[0] = {"joint1", 1.0, 0.0, 0.0, 0.0, -3.1416, 3.1416, "revolute"};
  joints[1] = {"joint2", 1.0, 0.0, 0.0, 0.0, -3.1416, 3.1416, "revolute"};
  RobotModel model;
  model.setJoints(joints);
  return model;
}

// Create the default 6-DOF PUMA-like arm.
RobotModel make6DOF()
{
  constexpr double kPi = 3.141592653589793;
  std::vector<DHJoint> joints(6);
  joints[0] = {"joint1", 0.0, 0.30, kPi / 2, 0.0, -kPi, kPi, "revolute"};
  joints[1] = {"joint2", 0.30, 0.0, 0.0, 0.0, -2.0944, 2.0944, "revolute"};
  joints[2] = {"joint3", 0.10, 0.0, -kPi / 2, 0.0, -2.0944, 2.0944, "revolute"};
  joints[3] = {"joint4", 0.0, 0.30, kPi / 2, 0.0, -kPi, kPi, "revolute"};
  joints[4] = {"joint5", 0.0, 0.0, -kPi / 2, 0.0, -2.0944, 2.0944, "revolute"};
  joints[5] = {"joint6", 0.0, 0.15, 0.0, 0.0, -kPi, kPi, "revolute"};
  RobotModel model;
  model.setJoints(joints);
  return model;
}

// ═══════════════════════════════════════════════════════════════
//  RobotModel — Construction & accessors
// ═══════════════════════════════════════════════════════════════

TEST(RobotModelTest, NumJoints2DOF)
{
  const auto model = makePlanar2DOF();
  EXPECT_EQ(model.numJoints(), 2U);
}

TEST(RobotModelTest, NumJoints6DOF)
{
  const auto model = make6DOF();
  EXPECT_EQ(model.numJoints(), 6U);
}

TEST(RobotModelTest, JointAccessor)
{
  const auto model = makePlanar2DOF();
  EXPECT_EQ(model.joint(0).name, "joint1");
  EXPECT_DOUBLE_EQ(model.joint(0).a, 1.0);
  EXPECT_DOUBLE_EQ(model.joint(1).a, 1.0);
}

TEST(RobotModelTest, ThrowsOnEmptyJoints)
{
  RobotModel model;
  EXPECT_THROW(model.setJoints({}), std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════════
//  Forward Kinematics
// ═══════════════════════════════════════════════════════════════

TEST(RobotModelTest, FKZeroPosition2DOF)
{
  // q = (0, 0) → arm fully extended along x-axis, tip at x=2.0
  const auto model = makePlanar2DOF();
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
  const Eigen::Matrix4d T = model.forwardKinematics(q);

  EXPECT_NEAR(T(0, 3), 2.0, 1e-10);  // x = a1 + a2 = 2.0
  EXPECT_NEAR(T(1, 3), 0.0, 1e-10);  // y = 0
  EXPECT_NEAR(T(2, 3), 0.0, 1e-10);  // z = 0
}

TEST(RobotModelTest, FKThrowsOnWrongSize)
{
  const auto model = makePlanar2DOF();
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(3);
  EXPECT_THROW(model.forwardKinematics(q), std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════════
//  Jacobian
// ═══════════════════════════════════════════════════════════════

TEST(RobotModelTest, JacobianDimensions2DOF)
{
  const auto model = makePlanar2DOF();
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
  const Eigen::MatrixXd J = model.numericalJacobian(q);

  EXPECT_EQ(J.rows(), 6);  // 3 linear + 3 angular
  EXPECT_EQ(J.cols(), 2);  // 2 joints
}

TEST(RobotModelTest, JacobianDimensions6DOF)
{
  const auto model = make6DOF();
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  const Eigen::MatrixXd J = model.numericalJacobian(q);

  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), 6);
}

// ═══════════════════════════════════════════════════════════════
//  Clamping
// ═══════════════════════════════════════════════════════════════

TEST(RobotModelTest, ClampWithinLimits)
{
  const auto model = makePlanar2DOF();
  Eigen::VectorXd q(2);
  q << 0.5, -0.5;
  const Eigen::VectorXd clamped = model.clampJoints(q);
  EXPECT_DOUBLE_EQ(clamped(0), 0.5);
  EXPECT_DOUBLE_EQ(clamped(1), -0.5);
}

TEST(RobotModelTest, ClampExceedsLimits)
{
  const auto model = makePlanar2DOF();
  Eigen::VectorXd q(2);
  q << 10.0, -10.0;  // far outside [-pi, pi]
  const Eigen::VectorXd clamped = model.clampJoints(q);
  EXPECT_LE(clamped(0), 3.1416);
  EXPECT_GE(clamped(1), -3.1416);
}

// ═══════════════════════════════════════════════════════════════
//  Inverse Kinematics — 2-DOF planar
// ═══════════════════════════════════════════════════════════════

TEST(RobotModelTest, IKConvergesToKnownPose2DOF)
{
  // Target: arm extended along x → tip at (2.0, 0, 0)
  const auto model = makePlanar2DOF();
  const Eigen::VectorXd q_init = Eigen::VectorXd::Zero(2);

  Eigen::Matrix4d T_desired = Eigen::Matrix4d::Identity();
  T_desired(0, 3) = 2.0;

  Eigen::VectorXd q_result;
  const bool ok = model.solveIK(q_init, T_desired, q_result,
    200, 1e-6, 1e-4, 0.1, 5);

  EXPECT_TRUE(ok) << "IK should converge for fully-extended arm";
  if (ok) {
    // Verify that forward kinematics of result matches target
    const Eigen::Matrix4d T_check = model.forwardKinematics(q_result);
    EXPECT_NEAR(T_check(0, 3), 2.0, 1e-3);
    EXPECT_NEAR(T_check(1, 3), 0.0, 1e-3);
    EXPECT_NEAR(T_check(2, 3), 0.0, 1e-3);
  }
}

// ═══════════════════════════════════════════════════════════════
//  Inverse Kinematics — 6-DOF PUMA
// ═══════════════════════════════════════════════════════════════

TEST(RobotModelTest, IKConverges6DOFFromHome)
{
  const auto model = make6DOF();
  // Home pose (all zeros): compute via FK, then solve IK back to it
  const Eigen::VectorXd q_home = Eigen::VectorXd::Zero(6);
  const Eigen::Matrix4d T_home = model.forwardKinematics(q_home);

  // Start from a perturbed configuration
  Eigen::VectorXd q_init(6);
  q_init << 0.1, -0.1, 0.2, -0.2, 0.1, -0.1;

  Eigen::VectorXd q_result;
  const bool ok = model.solveIK(q_init, T_home, q_result,
    200, 1e-6, 1e-4, 0.1, 5);

  EXPECT_TRUE(ok) << "IK should converge to home pose from nearby start";
  if (ok) {
    const Eigen::Matrix4d T_check = model.forwardKinematics(q_result);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(T_check(i, 3), T_home(i, 3), 1e-3);
    }
  }
}

}  // namespace ik_arm

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
