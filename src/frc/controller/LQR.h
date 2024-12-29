// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <frc/DARE.h>

namespace frc {

/**
 * Computes the infinite-horizon linear-quadratic regulator (LQR) feedback gain.
 *
 * The solver may hang if any of the following occur:
 * <ul>
 *   <li>Q isn't symmetric positive semidefinite</li>
 *   <li>R isn't symmetric positive definite</li>
 *   <li>The (A, B) pair isn't stabilizable</li>
 *   <li>The (A, C) pair where Q = CᵀC isn't detectable</li>
 * </ul>
 * Only use this function if you're sure the preconditions are met.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R The input cost matrix.
 * @return LQR feedback gain.
 */
template <int States, int Inputs>
Eigen::Matrix<double, Inputs, States> LQR(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R) {
  auto S = DARE(A, B, Q, R);

  // K = (BᵀSB + R)⁻¹BᵀSA
  return (B.transpose() * S * B + R).llt().solve(B.transpose() * S * A);
}

}  // namespace frc
