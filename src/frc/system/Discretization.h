// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

namespace frc {

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

}  // namespace frc
