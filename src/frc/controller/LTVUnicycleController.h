// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <wpi/interpolating_map.h>

#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/geometry/Pose2d.h"

namespace frc {

/**
 * The linear time-varying unicycle controller has a similar form to the LQR,
 * but the model used to compute the controller gain is the nonlinear unicycle
 * model linearized around the drivetrain's current state.
 *
 * This controller is a roughly drop-in replacement for RamseteController with
 * more optimal feedback gains in the "least-squares error" sense.
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
   * @param dt Discretization timestep.
   * @param maxVelocity The maximum velocity for the controller gain lookup
   *                    table.
   * @throws std::domain_error if maxVelocity &lt;= 0.
   */
  explicit LTVUnicycleController(double dt, double maxVelocity = 9.0);

  /**
   * Constructs a linear time-varying unicycle controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param Qelems The maximum desired error tolerance for each state.
   * @param Relems The maximum desired control effort for each input.
   * @param dt     Discretization timestep.
   * @param maxVelocity The maximum velocity for the controller gain lookup
   *                    table.
   * @throws std::domain_error if maxVelocity <= 0 m/s or >= 15 m/s.
   */
  LTVUnicycleController(const std::array<double, 3>& Qelems,
                        const std::array<double, 2>& Relems, double dt,
                        double maxVelocity = 9.0);

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
    const auto& eTranslate = m_poseError.Translation();
    const auto& eRotate = m_poseError.Rotation();
    const auto& tolTranslate = m_poseTolerance.Translation();
    const auto& tolRotate = m_poseTolerance.Rotation();
    return std::abs(eTranslate.X()) < tolTranslate.X() &&
           std::abs(eTranslate.Y()) < tolTranslate.Y() &&
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
  // LUT from drivetrain linear velocity to LQR gain
  wpi::interpolating_map<double, Eigen::Matrix<double, 2, 3>> m_table;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;
  bool m_enabled = true;
};

}  // namespace frc
