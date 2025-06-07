// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/LTVDifferentialDriveController.h"

#include <cmath>

#include "frc/AngleModulus.h"
#include "frc/LQR.h"
#include "frc/DiscretizeAB.h"

using namespace frc;

DifferentialDriveWheelVoltages LTVDifferentialDriveController::Calculate(
    const Pose2d& currentPose, double leftVelocity, double rightVelocity,
    const Pose2d& poseRef, double leftVelocityRef, double rightVelocityRef) {
  // This implements the linear time-varying differential drive controller in
  // theorem 8.7.4 of https://controls-in-frc.link/
  //
  //     [x ]
  //     [y ]       [Vₗ]
  // x = [θ ]   u = [Vᵣ]
  //     [vₗ]
  //     [vᵣ]

  double velocity = (leftVelocity + rightVelocity) / 2.0;

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (std::abs(velocity) < 1e-4) {
    velocity = 1e-4;
  }

  Eigen::Vector<double, 5> r{poseRef.translation.x, poseRef.translation.y,
                             poseRef.rotation.Radians(), leftVelocityRef,
                             rightVelocityRef};
  Eigen::Vector<double, 5> x{
      currentPose.translation.x, currentPose.translation.y,
      currentPose.rotation.Radians(), leftVelocity, rightVelocity};

  m_error = r - x;
  m_error(2) = frc::AngleModulus(m_error(2));

  Eigen::Matrix<double, 5, 5> A{
      {0.0, 0.0, 0.0, 0.5, 0.5},
      {0.0, 0.0, velocity, 0.0, 0.0},
      {0.0, 0.0, 0.0, -1.0 / m_trackwidth, 1.0 / m_trackwidth},
      {0.0, 0.0, 0.0, m_A(0, 0), m_A(0, 1)},
      {0.0, 0.0, 0.0, m_A(1, 0), m_A(1, 1)}};
  Eigen::Matrix<double, 5, 2> B{{0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {m_B(0, 0), m_B(0, 1)},
                                {m_B(1, 0), m_B(1, 1)}};

  Eigen::Matrix<double, 5, 5> discA;
  Eigen::Matrix<double, 5, 2> discB;
  DiscretizeAB(A, B, m_dt, &discA, &discB);

  auto K = LQR(discA, discB, m_Q, m_R);

  Eigen::Matrix<double, 5, 5> inRobotFrame{
      {std::cos(x(2)), std::sin(x(2)), 0.0, 0.0, 0.0},
      {-std::sin(x(2)), std::cos(x(2)), 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 1.0}};

  Eigen::Vector2d u = K * inRobotFrame * m_error;

  return {u(0), u(1)};
}
