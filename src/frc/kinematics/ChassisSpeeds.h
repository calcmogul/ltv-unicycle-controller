// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace frc {

/**
 * Represents the speed of a robot chassis. Although this struct contains
 * similar members compared to a Twist2d, they do NOT represent the same thing.
 * Whereas a Twist2d represents a change in pose w.r.t to the robot frame of
 * reference, a ChassisSpeeds struct represents a robot's velocity.
 *
 * A strictly non-holonomic drivetrain, such as a differential drive, should
 * never have a dy component because it can never move sideways. Holonomic
 * drivetrains such as swerve and mecanum will often have all three components.
 */
struct ChassisSpeeds {
  /**
   * Velocity along the x-axis. (Fwd is +)
   */
  double vx = 0.0;

  /**
   * Velocity along the y-axis. (Left is +)
   */
  double vy = 0.0;

  /**
   * Represents the angular velocity of the robot frame. (CCW is +)
   */
  double omega = 0.0;
};

}  // namespace frc
