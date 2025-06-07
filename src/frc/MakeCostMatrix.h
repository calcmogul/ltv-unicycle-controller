// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <limits>

#include <Eigen/Core>

namespace frc {

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

}  // namespace frc
