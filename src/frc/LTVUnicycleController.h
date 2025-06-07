// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Core>

#include "frc/Geometry2d.h"

namespace frc {

/**
 * Represents the speed of a robot chassis.
 */
struct ChassisSpeeds {
  /**
   * Velocity along the x-axis. (Fwd is +)
   */
  double vx = 0.0;

  /**
   * Velocity along the y-axis. (Left is +)
   */
  double vy = 0.0;

  /**
   * Represents the angular velocity of the robot frame. (CCW is +)
   */
  double omega = 0.0;
};

/**
 * The linear time-varying unicycle controller has a similar form to the LQR,
 * but the model used to compute the controller gain is the nonlinear unicycle
 * model linearized around the drivetrain's current state.
 *
 * See section 8.9 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.9.1.
 */
class LTVUnicycleController {
 public:
  /**
   * Constructs a linear time-varying unicycle controller with default maximum
   * desired error tolerances of (0.0625 m, 0.125 m, 2 rad) and default maximum
   * desired control effort of (1 m/s, 2 rad/s).
   *
   * @param dt Discretization timestep in seconds.
   */
  explicit LTVUnicycleController(double dt)
      : LTVUnicycleController{{0.0625, 0.125, 2.0}, {1.0, 2.0}, dt} {}

  /**
   * Constructs a linear time-varying unicycle controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param Qelems The maximum desired error tolerance for each state.
   * @param Relems The maximum desired control effort for each input.
   * @param dt     Discretization timestep in seconds.
   */
  LTVUnicycleController(const std::array<double, 3>& Qelems,
                        const std::array<double, 2>& Relems, double dt)
      : m_Q{MakeCostMatrix(Qelems)}, m_R{MakeCostMatrix(Relems)}, m_dt{dt} {}

  /**
   * Move constructor.
   */
  LTVUnicycleController(LTVUnicycleController&&) = default;

  /**
   * Move assignment operator.
   */
  LTVUnicycleController& operator=(LTVUnicycleController&&) = default;

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  bool AtReference() const {
    const auto& eTranslate = m_poseError.translation;
    const auto& eRotate = m_poseError.rotation;
    const auto& tolTranslate = m_poseTolerance.translation;
    const auto& tolRotate = m_poseTolerance.rotation;
    return std::abs(eTranslate.x) < tolTranslate.x &&
           std::abs(eTranslate.y) < tolTranslate.y &&
           std::abs(eRotate.Radians()) < tolRotate.Radians();
  }

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   */
  void SetTolerance(const Pose2d& poseTolerance) {
    m_poseTolerance = poseTolerance;
  }

  /**
   * Returns the linear and angular velocity outputs of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose        The current pose.
   * @param poseRef            The desired pose.
   * @param linearVelocityRef  The desired linear velocity.
   * @param angularVelocityRef The desired angular velocity.
   */
  ChassisSpeeds Calculate(const Pose2d& currentPose, const Pose2d& poseRef,
                          double linearVelocityRef, double angularVelocityRef);

  /**
   * Enables and disables the controller for troubleshooting purposes.
   *
   * @param enabled If the controller is enabled or not.
   */
  void SetEnabled(bool enabled) { m_enabled = enabled; }

 private:
  // LQR cost matrices
  Eigen::Matrix<double, 3, 3> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  double m_dt;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;
  bool m_enabled = true;

  /**
   * Creates a cost matrix from the given vector for use with LQR.
   *
   * The cost matrix is constructed using Bryson's rule. The inverse square of
   * each element in the input is placed on the cost matrix diagonal. If a
   * tolerance is infinity, its cost matrix entry is set to zero.
   *
   * @param costs An array. For a Q matrix, its elements are the maximum allowed
   *              excursions of the states from the reference. For an R matrix,
   *              its elements are the maximum allowed excursions of the control
   *              inputs from no actuation.
   * @return State excursion or control effort cost matrix.
   */
  template <size_t N>
  static constexpr Eigen::Matrix<double, N, N> MakeCostMatrix(
      const std::array<double, N>& costs) {
    Eigen::Matrix<double, N, N> result;

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        if (row == col) {
          if (costs[row] == std::numeric_limits<double>::infinity()) {
            result.coeffRef(row, col) = 0.0;
          } else {
            result.coeffRef(row, col) = 1.0 / (costs[row] * costs[row]);
          }
        } else {
          result.coeffRef(row, col) = 0.0;
        }
      }
    }

    return result;
  }
};

}  // namespace frc
