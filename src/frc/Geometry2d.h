// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

namespace frc {

/**
 * A rotation in a 2D coordinate frame represented by a point on the unit circle
 * (cosine and sine).
 */
class Rotation2d {
 public:
  /**
   * Constructs a Rotation2d with a default angle of 0 degrees.
   */
  constexpr Rotation2d() = default;

  /**
   * Constructs a Rotation2d with the given angle in radians.
   *
   * @param value The value of the angle in radians.
   */
  constexpr Rotation2d(double value)  // NOLINT
      : m_cos{std::cos(value)}, m_sin{std::sin(value)} {}

  /**
   * Constructs a Rotation2d with the given cosine and sine components.
   *
   * @param cos The cosine of the rotation.
   * @param sin The sine of the rotation.
   */
  constexpr Rotation2d(double cos, double sin) : m_cos{cos}, m_sin{sin} {}

  constexpr Rotation2d operator+(const Rotation2d& other) const {
    // [cos_new]   [other.cos, -other.sin][cos]
    // [sin_new] = [other.sin,  other.cos][sin]
    return {Cos() * other.Cos() - Sin() * other.Sin(),
            Cos() * other.Sin() + Sin() * other.Cos()};
  }
  constexpr Rotation2d operator-(const Rotation2d& other) const {
    return *this + (-other);
  }
  constexpr Rotation2d operator-() const { return Rotation2d{m_cos, -m_sin}; }

  constexpr double Radians() const { return std::atan2(m_sin, m_cos); }
  constexpr double Degrees() const {
    constexpr double pi = 3.141592653589793238462643383279502884L;
    return Radians() / pi * 180.0;
  }

  constexpr double Cos() const { return m_cos; }
  constexpr double Sin() const { return m_sin; }

 private:
  double m_cos = 1.0;
  double m_sin = 0.0;
};

/**
 * Represents a translation in 2D space (North-West-Up convention).
 */
struct Translation2d {
  double x = 0.0;
  double y = 0.0;

  constexpr Translation2d RotateBy(const Rotation2d& other) const {
    // [x_new]   [other.cos, -other.sin][x]
    // [y_new] = [other.sin,  other.cos][y]
    return {x * other.Cos() - y * other.Sin(),
            x * other.Sin() + y * other.Cos()};
  }

  constexpr Translation2d operator+(const Translation2d& other) const {
    return {x + other.x, y + other.y};
  }

  constexpr Translation2d operator-(const Translation2d& other) const {
    return {x - other.x, y - other.y};
  }
};

/**
 * Represents a 2D pose containing translational and rotational elements.
 */
struct Pose2d {
  Translation2d translation;
  Rotation2d rotation;

  /**
   * Returns this pose relative to another one.
   *
   * @param other The other pose.
   */
  constexpr Pose2d RelativeTo(const Pose2d& other) const {
    return {(translation - other.translation).RotateBy(-other.rotation),
            rotation - other.rotation};
  }
};

}  // namespace frc
