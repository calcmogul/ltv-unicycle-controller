// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <stdexcept>

#include "frc/DCMotor.h"
#include "frc/LinearSystem.h"

namespace frc {

/**
 * Identify a differential drive drivetrain given the drivetrain's kv and ka
 * in both linear {(volts/(meter/sec), (volts/(meter/sec²))} and angular
 * {(volts/(radian/sec), (volts/(radian/sec²))} cases. These constants can be
 * found using SysId.
 *
 * States: [[left velocity], [right velocity]]<br>
 * Inputs: [[left voltage], [right voltage]]<br>
 * Outputs: [[left velocity], [right velocity]]
 *
 * @param linear_kv The linear velocity gain in volts per (meters per second).
 * @param linear_ka The linear acceleration gain in volts per (meters per
 *                  second squared).
 * @param angular_kv The angular velocity gain in volts per (meters per
 *                   second).
 * @param angular_ka The angular acceleration gain in volts per (meters per
 *                   second squared).
 * @throws domain_error if linear_kv <= 0, linear_ka <= 0, angular_kv <= 0,
 *         or angular_ka <= 0.
 * @see <a
 * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
 */
inline LinearSystem<2, 2, 2> IdentifyDrivetrainSystem(double linear_kv,
                                                      double linear_ka,
                                                      double angular_kv,
                                                      double angular_ka) {
  if (linear_kv <= decltype(linear_kv){0}) {
    throw std::domain_error("Kv,linear must be greater than zero.");
  }
  if (linear_ka <= decltype(linear_ka){0}) {
    throw std::domain_error("Ka,linear must be greater than zero.");
  }
  if (angular_kv <= decltype(angular_kv){0}) {
    throw std::domain_error("Kv,angular must be greater than zero.");
  }
  if (angular_ka <= decltype(angular_ka){0}) {
    throw std::domain_error("Ka,angular must be greater than zero.");
  }

  double A1 = -(linear_kv / linear_ka + angular_kv / angular_ka);
  double A2 = -(linear_kv / linear_ka - angular_kv / angular_ka);
  double B1 = 1.0 / linear_ka + 1.0 / angular_ka;
  double B2 = 1.0 / linear_ka - 1.0 / angular_ka;

  A1 /= 2.0;
  A2 /= 2.0;
  B1 /= 2.0;
  B2 /= 2.0;

  Eigen::Matrix2d A{{A1, A2}, {A2, A1}};
  Eigen::Matrix2d B{{B1, B2}, {B2, B1}};
  Eigen::Matrix2d C{{1.0, 0.0}, {0.0, 1.0}};
  Eigen::Matrix2d D{{0.0, 0.0}, {0.0, 0.0}};

  return LinearSystem<2, 2, 2>{A, B, C, D};
}

/**
 * Identify a differential drive drivetrain given the drivetrain's kV and kA
 * in both linear {(volts/(meter/sec)), (volts/(meter/sec²))} and angular
 * {(volts/(radian/sec)), (volts/(radian/sec²))} cases. This can be found
 * using SysId.
 *
 * States: [[left velocity], [right velocity]]<br>
 * Inputs: [[left voltage], [right voltage]]<br>
 * Outputs: [[left velocity], [right velocity]]
 *
 * @param linear_kv The linear velocity gain in volts per (meters per second).
 * @param linear_ka The linear acceleration gain in volts per (meters per
 *                  second squared).
 * @param angular_kv The angular velocity gain in volts per (radians per
 *                   second).
 * @param angular_ka The angular acceleration gain in volts per (radians per
 *                   second squared).
 * @param trackwidth The distance between the differential drive's left and
 *                   right wheels, in meters.
 * @throws domain_error if linear_kv <= 0, linear_ka <= 0, angular_kv <= 0,
 *         angular_ka <= 0, or trackwidth <= 0.
 * @see <a
 * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
 */
inline LinearSystem<2, 2, 2> IdentifyDrivetrainSystem(double linear_kv,
                                                      double linear_ka,
                                                      double angular_kv,
                                                      double angular_ka,
                                                      double trackwidth) {
  if (linear_kv <= 0.0) {
    throw std::domain_error("Kv,linear must be greater than zero.");
  }
  if (linear_ka <= 0.0) {
    throw std::domain_error("Ka,linear must be greater than zero.");
  }
  if (angular_kv <= 0.0) {
    throw std::domain_error("Kv,angular must be greater than zero.");
  }
  if (angular_ka <= 0.0) {
    throw std::domain_error("Ka,angular must be greater than zero.");
  }
  if (trackwidth <= 0.0) {
    throw std::domain_error("r must be greater than zero.");
  }

  // We want to find a factor to include in Kv,angular that will convert
  // `u = Kv,angular omega` to `u = Kv,angular v`.
  //
  // v = omega r
  // omega = v/r
  // omega = 1/r v
  // omega = 1/(trackwidth/2) v
  // omega = 2/trackwidth v
  //
  // So multiplying by 2/trackwidth converts the angular gains from V/(rad/s)
  // to V/(m/s).
  return IdentifyDrivetrainSystem(linear_kv, linear_ka,
                                  angular_kv * 2.0 / trackwidth,
                                  angular_ka * 2.0 / trackwidth);
}

/**
 * Create a state-space model of differential drive drivetrain. In this model,
 * the states are [left velocity, right velocity]ᵀ, the inputs are [left
 * voltage, right voltage], and the outputs are [left velocity, right
 * velocity]ᵀ.
 *
 * @param motor The motor (or gearbox) driving the drivetrain.
 * @param mass The mass of the robot in kilograms.
 * @param r The radius of the wheels in meters.
 * @param rb The radius of the base (half of the trackwidth), in meters.
 * @param J The moment of inertia of the robot.
 * @param gearing Gear ratio from motor to wheel.
 * @throws std::domain_error if mass <= 0, r <= 0, rb <= 0, J <= 0, or
 *         gearing <= 0.
 */
inline LinearSystem<2, 2, 2> DrivetrainVelocitySystem(const DCMotor& motor,
                                                      double mass, double r,
                                                      double rb, double J,
                                                      double gearing) {
  if (mass <= 0.0) {
    throw std::domain_error("mass must be greater than zero.");
  }
  if (r <= 0.0) {
    throw std::domain_error("r must be greater than zero.");
  }
  if (rb <= 0.0) {
    throw std::domain_error("rb must be greater than zero.");
  }
  if (J <= 0.0) {
    throw std::domain_error("J must be greater than zero.");
  }
  if (gearing <= 0.0) {
    throw std::domain_error("gearing must be greater than zero.");
  }

  auto C1 = -std::pow(gearing, 2) * motor.Kt / (motor.Kv * motor.R * r * r);
  auto C2 = gearing * motor.Kt / (motor.R * r);

  Eigen::Matrix2d A{
      {((1 / mass + rb * rb / J) * C1), ((1 / mass - rb * rb / J) * C1)},
      {((1 / mass - rb * rb / J) * C1), ((1 / mass + rb * rb / J) * C1)}};
  Eigen::Matrix2d B{
      {((1 / mass + rb * rb / J) * C2), ((1 / mass - rb * rb / J) * C2)},
      {((1 / mass - rb * rb / J) * C2), ((1 / mass + rb * rb / J) * C2)}};
  Eigen::Matrix2d C{{1.0, 0.0}, {0.0, 1.0}};
  Eigen::Matrix2d D{{0.0, 0.0}, {0.0, 0.0}};

  return LinearSystem<2, 2, 2>{A, B, C, D};
}

}  // namespace frc
