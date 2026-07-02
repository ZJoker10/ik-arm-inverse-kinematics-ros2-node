#include "ik_arm/robot_model.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace ik_arm
{

// ═══════════════════════════════════════════════════════════════
//  DH Transform
// ═══════════════════════════════════════════════════════════════

Eigen::Matrix4d RobotModel::dhTransform(
  double theta, double d, double a, double alpha)
{
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);

  Eigen::Matrix4d T;
  // clang-format off
  T << ct, -st * ca, st * sa, a * ct,
    st, ct * ca, -ct * sa, a * st,
    0, sa, ca, d,
    0, 0, 0, 1;
  // clang-format on
  return T;
}

// ═══════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════

void RobotModel::setJoints(const std::vector<DHJoint> & joints)
{
  if (joints.empty()) {
    throw std::invalid_argument("RobotModel: at least one joint required");
  }
  joints_ = joints;
}

// ═══════════════════════════════════════════════════════════════
//  Forward Kinematics
// ═══════════════════════════════════════════════════════════════

Eigen::Matrix4d RobotModel::forwardKinematics(const Eigen::VectorXd & q) const
{
  if (static_cast<size_t>(q.size()) != joints_.size()) {
    throw std::invalid_argument(
      "forwardKinematics: q size must match number of joints");
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    const double theta = q(i) + j.theta_offset;
    T = T * dhTransform(theta, j.d, j.a, j.alpha);
  }
  return T;
}

std::vector<Eigen::Matrix4d> RobotModel::allLinkTransforms(
  const Eigen::VectorXd & q) const
{
  const size_t n = joints_.size();
  std::vector<Eigen::Matrix4d> transforms(n + 1);
  transforms[0] = Eigen::Matrix4d::Identity();

  for (size_t i = 0; i < n; ++i) {
    const auto & j = joints_[i];
    const double theta = q(i) + j.theta_offset;
    transforms[i + 1] =
      transforms[i] * dhTransform(theta, j.d, j.a, j.alpha);
  }
  return transforms;
}

// ═══════════════════════════════════════════════════════════════
//  Numerical Jacobian (6×n)
// ═══════════════════════════════════════════════════════════════

Eigen::MatrixXd RobotModel::numericalJacobian(
  const Eigen::VectorXd & q) const
{
  const size_t n = joints_.size();
  constexpr double kEps = 1e-8;

  // Baseline forward kinematics
  const Eigen::Matrix4d T0 = forwardKinematics(q);
  const Eigen::Vector3d p0 = T0.block<3, 1>(0, 3);
  const Eigen::Matrix3d R0 = T0.block<3, 3>(0, 0);

  Eigen::MatrixXd J(6, n);

  for (size_t i = 0; i < n; ++i) {
    // Perturb joint i
    Eigen::VectorXd q_pert = q;
    q_pert(i) += kEps;

    const Eigen::Matrix4d Tp = forwardKinematics(q_pert);
    const Eigen::Vector3d pp = Tp.block<3, 1>(0, 3);
    const Eigen::Matrix3d Rp = Tp.block<3, 3>(0, 0);

    // Linear velocity part (position)
    J.block<3, 1>(0, i) = (pp - p0) / kEps;

    // Angular velocity part (orientation)
    //   R_diff = Rp * R0^T  ≈  I + skew(ω)
    //   Extract ω from skew-symmetric part of R_diff
    const Eigen::Matrix3d R_diff = Rp * R0.transpose();
    Eigen::Vector3d omega;
    omega << R_diff(2, 1) - R_diff(1, 2),
      R_diff(0, 2) - R_diff(2, 0),
      R_diff(1, 0) - R_diff(0, 1);
    omega /= (2.0 * kEps);

    J.block<3, 1>(3, i) = omega;
  }

  return J;
}

// ═══════════════════════════════════════════════════════════════
//  Inverse Kinematics (Damped Pseudo-Inverse / Levenberg-Marquardt)
// ═══════════════════════════════════════════════════════════════

bool RobotModel::solveIK(
  const Eigen::VectorXd & q_init,
  const Eigen::Matrix4d & T_desired,
  Eigen::VectorXd & q_result,
  int max_iters,
  double pos_tol,
  double orient_tol,
  double lambda,
  int max_retries) const
{
  constexpr double kDt = 0.5;

  const Eigen::Vector3d p_des = T_desired.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_des = T_desired.block<3, 3>(0, 0);

  // Best result across all retries
  Eigen::VectorXd best_q = clampJoints(q_init);
  double best_pos_err = std::numeric_limits<double>::max();

  for (int retry = 0; retry <= max_retries; ++retry) {
    Eigen::VectorXd q =
      (retry == 0) ? clampJoints(q_init) : randomConfiguration();

    for (int iter = 0; iter < max_iters; ++iter) {
      const Eigen::Matrix4d T_cur = forwardKinematics(q);
      const Eigen::Vector3d p_cur = T_cur.block<3, 1>(0, 3);
      const Eigen::Matrix3d R_cur = T_cur.block<3, 3>(0, 0);

      // Position error
      const Eigen::Vector3d pos_error = p_des - p_cur;

      // Orientation error (angle-axis)
      const Eigen::Matrix3d R_err = R_des * R_cur.transpose();
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

      const double pos_err_norm = pos_error.norm();
      const double orient_err_norm = orient_error.norm();

      // Track best across retries
      if (pos_err_norm < best_pos_err) {
        best_pos_err = pos_err_norm;
        best_q = q;
      }

      // Convergence check
      if (pos_err_norm < pos_tol && orient_err_norm < orient_tol) {
        q_result = q;
        return true;
      }

      // SVD damped pseudo-inverse
      const Eigen::MatrixXd J = numericalJacobian(q);
      Eigen::VectorXd e(6);
      e.head<3>() = pos_error;
      e.tail<3>() = orient_error;

      Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J, Eigen::ComputeFullU | Eigen::ComputeFullV);
      const Eigen::VectorXd sv = svd.singularValues();
      Eigen::VectorXd dq = Eigen::VectorXd::Zero(J.cols());
      for (int i = 0; i < sv.size(); ++i) {
        const double s = sv(i);
        dq += (s / (s * s + lambda * lambda)) *
          svd.matrixU().col(i).dot(e) * svd.matrixV().col(i);
      }

      q += kDt * dq;
      q = clampJoints(q);
    }
  }

  // Return best result even if not converged
  q_result = best_q;
  return false;
}

// ═══════════════════════════════════════════════════════════════
//  Utilities
// ═══════════════════════════════════════════════════════════════

Eigen::VectorXd RobotModel::clampJoints(const Eigen::VectorXd & q) const
{
  Eigen::VectorXd clamped = q;
  for (size_t i = 0; i < joints_.size(); ++i) {
    clamped(i) = std::clamp(
      clamped(i), joints_[i].position_min, joints_[i].position_max);
  }
  return clamped;
}

Eigen::VectorXd RobotModel::randomConfiguration() const
{
  static std::random_device rd;
  static std::mt19937 gen(rd());

  Eigen::VectorXd q(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    std::uniform_real_distribution<double> dist(
      joints_[i].position_min, joints_[i].position_max);
    q(i) = dist(gen);
  }
  return q;
}

}  // namespace ik_arm
