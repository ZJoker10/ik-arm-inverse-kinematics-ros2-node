#ifndef IK_ARM__ROBOT_MODEL_HPP_
#define IK_ARM__ROBOT_MODEL_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace ik_arm
{
struct DHJoint
{
  std::string name;
  double a;
  double d;
  double alpha;
  double theta_offset;

  double position_min;
  double position_max;

  std::string joint_type;
};
class RobotModel
{
public:
  RobotModel() = default;

  /// Load DH parameters; clears any previous configuration.
  void setJoints(const std::vector<DHJoint> & joints);

  /// Number of joints (degrees of freedom).
  size_t numJoints() const {return joints_.size();}

  /// Read-only access to a specific joint.
  const DHJoint & joint(size_t i) const {return joints_.at(i);}

  // ── Forward Kinematics ──────────────────────────────────────

  Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd & q) const;

  std::vector<Eigen::Matrix4d> allLinkTransforms(
    const Eigen::VectorXd & q) const;

  // ── Jacobian ────────────────────────────────────────────────

  Eigen::MatrixXd numericalJacobian(const Eigen::VectorXd & q) const;

  // ── Inverse Kinematics ──────────────────────────────────────

  /**
   * @brief Solve IK via damped pseudo-inverse (Levenberg-Marquardt).
   *
   * Uses SVD: Δq = Σ_i σ_i/(σ_i²+λ²) · (U_i·e) · V_i.
   * Retries from random configurations on failure.
   *
   * @param q_init       Initial guess
   * @param T_desired    Target 4×4 transform
   * @param q_result     Output: solved angles
   * @param max_iters    Max Newton iterations per attempt
   * @param pos_tol      Position tolerance (m)
   * @param orient_tol   Orientation tolerance (rad)
   * @param lambda       Damping factor
   * @param max_retries  Random restarts if no convergence
   * @return             true if converged within tolerances
   */
  bool solveIK(
    const Eigen::VectorXd & q_init,
    const Eigen::Matrix4d & T_desired,
    Eigen::VectorXd & q_result,
    int max_iters = 200,
    double pos_tol = 1e-6,
    double orient_tol = 1e-4,
    double lambda = 0.1,
    int max_retries = 5) const;

  // ── Utilities ───────────────────────────────────────────────

  /// Clamp angles to joint limits.
  Eigen::VectorXd clampJoints(const Eigen::VectorXd & q) const;

  /// Random configuration within limits.
  Eigen::VectorXd randomConfiguration() const;

private:
  std::vector<DHJoint> joints_;

  /// Single DH transform (Craig convention).
  static Eigen::Matrix4d dhTransform(
    double theta, double d, double a, double alpha);
};

}  // namespace ik_arm

#endif  // IK_ARM__ROBOT_MODEL_HPP_
