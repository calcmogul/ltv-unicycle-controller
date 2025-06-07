// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/LTVUnicycleController.h"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <frc/LQR.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace {

/**
 * Discretizes the given continuous A and B matrices.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param contA Continuous system matrix.
 * @param contB Continuous input matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discB Storage for discrete input matrix.
 */
template <int States, int Inputs>
void DiscretizeAB(const Eigen::Matrix<double, States, States>& contA,
                  const Eigen::Matrix<double, States, Inputs>& contB, double dt,
                  Eigen::Matrix<double, States, States>* discA,
                  Eigen::Matrix<double, States, Inputs>* discB) {
  // M = [A  B]
  //     [0  0]
  Eigen::Matrix<double, States + Inputs, States + Inputs> M;
  M.template block<States, States>(0, 0) = contA;
  M.template block<States, Inputs>(0, States) = contB;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  // ϕ = eᴹᵀ = [A_d  B_d]
  //           [ 0    I ]
  Eigen::Matrix<double, States + Inputs, States + Inputs> phi = (M * dt).exp();

  *discA = phi.template block<States, States>(0, 0);
  *discB = phi.template block<States, Inputs>(0, States);
}

}  // namespace

namespace frc {

ChassisSpeeds LTVUnicycleController::Calculate(const Pose2d& currentPose,
                                               const Pose2d& poseRef,
                                               double linearVelocityRef,
                                               double angularVelocityRef) {
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

  if (!m_enabled) {
    return ChassisSpeeds{linearVelocityRef, 0.0, angularVelocityRef};
  }

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (std::abs(linearVelocityRef) < 1e-4) {
    linearVelocityRef = 1e-4;
  }

  m_poseError = poseRef.RelativeTo(currentPose);

  Eigen::Matrix<double, 3, 3> A{
      {0.0, 0.0, 0.0}, {0.0, 0.0, linearVelocityRef}, {0.0, 0.0, 0.0}};
  Eigen::Matrix<double, 3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};

  Eigen::Matrix<double, 3, 3> discA;
  Eigen::Matrix<double, 3, 2> discB;
  DiscretizeAB(A, B, m_dt, &discA, &discB);

  auto K = LQR(discA, discB, m_Q, m_R);

  Eigen::Vector3d e{m_poseError.translation.x, m_poseError.translation.y,
                    m_poseError.rotation.Radians()};
  Eigen::Vector2d u = K * e;

  return ChassisSpeeds{linearVelocityRef + u(0), 0.0,
                       angularVelocityRef + u(1)};
}

}  // namespace frc
