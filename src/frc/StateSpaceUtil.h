// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

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
constexpr Eigen::Matrix<double, N, N> MakeCostMatrix(
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

/**
 * Returns true if (A, B) is a stabilizable pair.
 *
 * (A, B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is
 * uncontrollable if rank([λI - A, B]) < n where n is the number of states.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param A System matrix.
 * @param B Input matrix.
 */
template <int States, int Inputs>
bool IsStabilizable(const Eigen::Matrix<double, States, States>& A,
                    const Eigen::Matrix<double, States, Inputs>& B) {
  Eigen::EigenSolver<Eigen::Matrix<double, States, States>> es{A, false};

  for (int i = 0; i < A.rows(); ++i) {
    if (std::norm(es.eigenvalues()[i]) < 1) {
      continue;
    }

    Eigen::Matrix<std::complex<double>, States, States + Inputs> E;
    E << es.eigenvalues()[i] * Eigen::Matrix<std::complex<double>, States,
                                             States>::Identity() -
             A,
        B;

    Eigen::ColPivHouseholderQR<
        Eigen::Matrix<std::complex<double>, States, States + Inputs>>
        qr{E};
    if (qr.rank() < States) {
      return false;
    }
  }
  return true;
}

/**
 * Returns true if (A, C) is a detectable pair.
 *
 * (A, C) is detectable if and only if the unobservable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is unobservable
 * if rank([λI - A; C]) < n where n is the number of states.
 *
 * @tparam States Number of states.
 * @tparam Outputs Number of outputs.
 * @param A System matrix.
 * @param C Output matrix.
 */
template <int States, int Outputs>
bool IsDetectable(const Eigen::Matrix<double, States, States>& A,
                  const Eigen::Matrix<double, Outputs, States>& C) {
  return IsStabilizable<States, Outputs>(A.transpose(), C.transpose());
}

}  // namespace frc
