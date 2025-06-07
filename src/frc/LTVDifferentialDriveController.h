// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "frc/MakeCostMatrix.h"
#include "frc/Geometry2d.h"
#include "frc/LinearSystem.h"

namespace frc {

/**
 * Motor voltages for a differential drive.
 */
struct DifferentialDriveWheelVoltages {
  /// Left wheel voltage.
  double left = 0.0;

  /// Right wheel voltage.
  double right = 0.0;
};

/**
 * The linear time-varying differential drive controller has a similar form to
 * the LQR, but the model used to compute the controller gain is the nonlinear
 * differential drive model linearized around the drivetrain's current state. We
 * precompute gains for important places in our state-space, then interpolate
 * between them with a lookup table to save computational resources.
 *
 * This controller has a flat hierarchy with pose and wheel velocity references
 * and voltage outputs. This is different from a unicycle controller's nested
 * hierarchy where the top-level controller has a pose reference and chassis
 * velocity command outputs, and the low-level controller has wheel velocity
 * references and voltage outputs. Flat hierarchies are easier to tune in one
 * shot.
 *
 * See section 8.7 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.7.4.
 */
class LTVDifferentialDriveController {
 public:
  /**
   * Constructs a linear time-varying differential drive controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param plant      The differential drive velocity plant.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param Qelems     The maximum desired error tolerance for each state.
   * @param Relems     The maximum desired control effort for each input.
   * @param dt         Discretization timestep.
   */
  LTVDifferentialDriveController(const LinearSystem<2, 2, 2>& plant,
                                 double trackwidth,
                                 const std::array<double, 5>& Qelems,
                                 const std::array<double, 2>& Relems, double dt)
      : m_trackwidth{trackwidth},
        m_A{plant.A},
        m_B{plant.B},
        m_Q{MakeCostMatrix(Qelems)},
        m_R{MakeCostMatrix(Relems)},
        m_dt{dt} {}

  /**
   * Move constructor.
   */
  LTVDifferentialDriveController(LTVDifferentialDriveController&&) = default;

  /**
   * Move assignment operator.
   */
  LTVDifferentialDriveController& operator=(LTVDifferentialDriveController&&) =
      default;

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  bool AtReference() const {
    return std::abs(m_error(0)) < m_tolerance(0) &&
           std::abs(m_error(1)) < m_tolerance(1) &&
           std::abs(m_error(2)) < m_tolerance(2) &&
           std::abs(m_error(3)) < m_tolerance(3) &&
           std::abs(m_error(4)) < m_tolerance(4);
  }

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   * @param leftVelocityTolerance Left velocity error which is tolerable.
   * @param rightVelocityTolerance Right velocity error which is tolerable.
   */
  void SetTolerance(const Pose2d& poseTolerance, double leftVelocityTolerance,
                    double rightVelocityTolerance) {
    m_tolerance = Eigen::Vector<double, 5>{
        poseTolerance.translation.x, poseTolerance.translation.y,
        poseTolerance.rotation.Radians(), leftVelocityTolerance,
        rightVelocityTolerance};
  }

  /**
   * Returns the left and right output voltages of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose      The current pose.
   * @param leftVelocity     The current left velocity.
   * @param rightVelocity    The current right velocity.
   * @param poseRef          The desired pose.
   * @param leftVelocityRef  The desired left velocity.
   * @param rightVelocityRef The desired right velocity.
   */
  DifferentialDriveWheelVoltages Calculate(
      const Pose2d& currentPose, double leftVelocity, double rightVelocity,
      const Pose2d& poseRef, double leftVelocityRef, double rightVelocityRef);

 private:
  double m_trackwidth;

  // Continuous velocity dynamics
  Eigen::Matrix<double, 2, 2> m_A;
  Eigen::Matrix<double, 2, 2> m_B;

  // LQR cost matrices
  Eigen::Matrix<double, 5, 5> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  double m_dt;

  Eigen::Vector<double, 5> m_error;
  Eigen::Vector<double, 5> m_tolerance;
};

}  // namespace frc
