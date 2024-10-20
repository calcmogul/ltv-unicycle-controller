// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/LTVUnicycleController.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>

#include "frc/DARE.h"
#include "frc/StateSpaceUtil.h"
#include "frc/system/Discretization.h"

using namespace frc;

namespace {

/**
 * States of the drivetrain system.
 */
class State {
 public:
  /// X position in global coordinate frame.
  [[maybe_unused]]
  static constexpr int kX = 0;

  /// Y position in global coordinate frame.
  static constexpr int kY = 1;

  /// Heading in global coordinate frame.
  static constexpr int kHeading = 2;
};

}  // namespace

LTVUnicycleController::LTVUnicycleController(double dt, double maxVelocity)
    : LTVUnicycleController{{0.0625, 0.125, 2.0}, {1.0, 2.0}, dt, maxVelocity} {
}

LTVUnicycleController::LTVUnicycleController(
    const std::array<double, 3>& Qelems, const std::array<double, 2>& Relems,
    double dt, double maxVelocity) {
  if (maxVelocity <= 0.0) {
    throw std::domain_error("Max velocity must be greater than 0 m/s.");
  }
  if (maxVelocity >= 15.0) {
    throw std::domain_error("Max velocity must be less than 15 m/s.");
  }

  // The change in global pose for a unicycle is defined by the following three
  // equations.
  //
  // ẋ = v cosθ
  // ẏ = v sinθ
  // θ̇ = ω
  //
  // Here's the model as a vector function where x = [x  y  θ]ᵀ and u = [v  ω]ᵀ.
  //
  //           [v cosθ]
  // f(x, u) = [v sinθ]
  //           [  ω   ]
  //
  // To create an LQR, we need to linearize this.
  //
  //               [0  0  −v sinθ]                  [cosθ  0]
  // ∂f(x, u)/∂x = [0  0   v cosθ]    ∂f(x, u)/∂u = [sinθ  0]
  //               [0  0     0   ]                  [ 0    1]
  //
  // We're going to make a cross-track error controller, so we'll apply a
  // clockwise rotation matrix to the global tracking error to transform it into
  // the robot's coordinate frame. Since the cross-track error is always
  // measured from the robot's coordinate frame, the model used to compute the
  // LQR should be linearized around θ = 0 at all times.
  //
  //     [0  0  −v sin0]        [cos0  0]
  // A = [0  0   v cos0]    B = [sin0  0]
  //     [0  0     0   ]        [ 0    1]
  //
  //     [0  0  0]              [1  0]
  // A = [0  0  v]          B = [0  0]
  //     [0  0  0]              [0  1]
  Eigen::Matrix<double, 3, 3> A = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};
  Eigen::Matrix<double, 3, 3> Q = frc::MakeCostMatrix(Qelems);
  Eigen::Matrix<double, 2, 2> R = frc::MakeCostMatrix(Relems);

  auto R_llt = R.llt();

  for (auto velocity = -maxVelocity; velocity < maxVelocity; velocity += 0.01) {
    // The DARE is ill-conditioned if the velocity is close to zero, so don't
    // let the system stop.
    if (std::abs(velocity) < 1e-4) {
      A(State::kY, State::kHeading) = 1e-4;
    } else {
      A(State::kY, State::kHeading) = velocity;
    }

    Eigen::Matrix<double, 3, 3> discA;
    Eigen::Matrix<double, 3, 2> discB;
    DiscretizeAB(A, B, dt, &discA, &discB);

    Eigen::Matrix<double, 3, 3> S = DARE<3, 2>(discA, discB, Q, R_llt);

    // K = (BᵀSB + R)⁻¹BᵀSA
    m_table.insert(velocity, (discB.transpose() * S * discB + R)
                                 .llt()
                                 .solve(discB.transpose() * S * discA));
  }
}

ChassisSpeeds LTVUnicycleController::Calculate(const Pose2d& currentPose,
                                               const Pose2d& poseRef,
                                               double linearVelocityRef,
                                               double angularVelocityRef) {
  if (!m_enabled) {
    return ChassisSpeeds{linearVelocityRef, 0.0, angularVelocityRef};
  }

  m_poseError = poseRef.RelativeTo(currentPose);

  const auto& K = m_table[linearVelocityRef];
  Eigen::Vector3d e{m_poseError.X(), m_poseError.Y(),
                    m_poseError.Rotation().Radians()};
  Eigen::Vector2d u = K * e;

  return ChassisSpeeds{linearVelocityRef + u(0), 0.0,
                       angularVelocityRef + u(1)};
}
