// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>

namespace frc {

/**
 * A plant defined using state-space notation.
 *
 * A plant is a mathematical model of a system's dynamics.
 *
 * For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @tparam Outputs Number of outputs.
 */
template <int States, int Inputs, int Outputs>
struct LinearSystem {
  /**
   * Continuous system matrix.
   */
  Eigen::Matrix<double, States, States> A;

  /**
   * Continuous input matrix.
   */
  Eigen::Matrix<double, States, Inputs> B;

  /**
   * Output matrix.
   */
  Eigen::Matrix<double, Outputs, States> C;

  /**
   * Feedthrough matrix.
   */
  Eigen::Matrix<double, Outputs, Inputs> D;
};

}  // namespace frc
