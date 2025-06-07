// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace frc {

/**
 * Returns modulus of input.
 *
 * @param input        Input value to wrap.
 * @param minimumInput The minimum value expected from the input.
 * @param maximumInput The maximum value expected from the input.
 */
template <typename T>
constexpr T InputModulus(T input, T minimumInput, T maximumInput) {
  T modulus = maximumInput - minimumInput;

  // Wrap input if it's above the maximum input
  int numMax = (input - minimumInput) / modulus;
  input -= numMax * modulus;

  // Wrap input if it's below the minimum input
  int numMin = (input - maximumInput) / modulus;
  input -= numMin * modulus;

  return input;
}

/**
 * Wraps an angle to the range -π to π radians.
 *
 * @param angle Angle to wrap.
 */
constexpr double AngleModulus(double angle) {
  constexpr double pi = 3.141592653589793238462643383279502884L;
  return InputModulus<double>(angle, -pi, pi);
}

}  // namespace frc
