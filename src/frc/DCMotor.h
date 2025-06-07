// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace frc {

/**
 * Holds the constants for a DC motor.
 */
class DCMotor {
 public:
  /// Voltage at which the motor constants were measured in V.
  double nominalVoltage;

  /// Torque when stalled in N-m.
  double stallTorque;

  /// Current draw when stalled in A.
  double stallCurrent;

  /// Current draw under no load in A.
  double freeCurrent;

  /// Angular velocity under no load in rad/s.
  double freeSpeed;

  /// Motor internal resistance in Ω.
  double R;

  /// Motor velocity constant in (rad/s)/V.
  double Kv;

  /// Motor torque constant in N-m/A.
  double Kt;

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltage Voltage at which the motor constants were measured.
   * @param stallTorque Torque when stalled.
   * @param stallCurrent Current draw when stalled.
   * @param freeCurrent Current draw under no load.
   * @param freeSpeed Angular velocity under no load.
   * @param numMotors Number of motors in a gearbox.
   */
  constexpr DCMotor(double nominalVoltage, double stallTorque,
                    double stallCurrent, double freeCurrent, double freeSpeed,
                    int numMotors = 1)
      : nominalVoltage(nominalVoltage),
        stallTorque(stallTorque * numMotors),
        stallCurrent(stallCurrent * numMotors),
        freeCurrent(freeCurrent * numMotors),
        freeSpeed(freeSpeed),
        R(nominalVoltage / this->stallCurrent),
        Kv(freeSpeed / (nominalVoltage - R * this->freeCurrent)),
        Kt(this->stallTorque / this->stallCurrent) {}

  /**
   * Returns current in A drawn by motor with given speed and input voltage.
   *
   * @param speed The current angular velocity of the motor in rad/s.
   * @param inputVoltage The voltage being applied to the motor in V.
   */
  constexpr double Current(double speed, double inputVoltage) const {
    return -1.0 / Kv / R * speed + 1.0 / R * inputVoltage;
  }

  /**
   * Returns current in A drawn by motor for a given torque.
   *
   * @param torque The torque produced by the motor in N-m.
   */
  constexpr double Current(double torque) const { return torque / Kt; }

  /**
   * Returns torque in N-m produced by the motor with a given current.
   *
   * @param current The current drawn by the motor in A.
   */
  constexpr double Torque(double current) const { return current * Kt; }

  /**
   * Returns the voltage in V provided to the motor for a given torque and
   * angular velocity.
   *
   * @param torque The torque produced by the motor in N-m.
   * @param speed The current angular velocity of the motor in rad/s.
   */
  constexpr double Voltage(double torque, double speed) const {
    return 1.0 / Kv * speed + 1.0 / Kt * R * torque;
  }

  /**
   * Returns the angular speed in rad/s produced by the motor at a given torque
   * and input voltage.
   *
   * @param torque The torque produced by the motor in N-m.
   * @param inputVoltage The input voltage provided to the motor in V.
   */
  constexpr double Speed(double torque, double inputVoltage) const {
    return inputVoltage * Kv - 1.0 / Kt * torque * R * Kv;
  }

  /**
   * Returns a copy of this motor with the given gearbox reduction applied.
   *
   * @param gearboxReduction  The gearbox reduction.
   */
  constexpr DCMotor WithReduction(double gearboxReduction) {
    return DCMotor(nominalVoltage, stallTorque * gearboxReduction, stallCurrent,
                   freeCurrent, freeSpeed / gearboxReduction);
  }

  /**
   * Returns a gearbox of CIM motors.
   */
  static constexpr DCMotor CIM(int numMotors = 1) {
    return DCMotor(12, 2.42, 133, 2.7, rpm2radpsec(5310), numMotors);
  }

  /**
   * Returns a gearbox of MiniCIM motors.
   */
  static constexpr DCMotor MiniCIM(int numMotors = 1) {
    return DCMotor(12, 1.41, 89, 3, rpm2radpsec(5840), numMotors);
  }

  /**
   * Returns a gearbox of Bag motor motors.
   */
  static constexpr DCMotor Bag(int numMotors = 1) {
    return DCMotor(12, 0.43, 53, 1.8, rpm2radpsec(13180), numMotors);
  }

  /**
   * Returns a gearbox of Vex 775 Pro motors.
   */
  static constexpr DCMotor Vex775Pro(int numMotors = 1) {
    return DCMotor(12, 0.71, 134, 0.7, rpm2radpsec(18730), numMotors);
  }

  /**
   * Returns a gearbox of Andymark RS 775-125 motors.
   */
  static constexpr DCMotor RS775_125(int numMotors = 1) {
    return DCMotor(12, 0.28, 18, 1.6, rpm2radpsec(5800), numMotors);
  }

  /**
   * Returns a gearbox of Banebots RS 775 motors.
   */
  static constexpr DCMotor BanebotsRS775(int numMotors = 1) {
    return DCMotor(12, 0.72, 97, 2.7, rpm2radpsec(13050), numMotors);
  }

  /**
   * Returns a gearbox of Andymark 9015 motors.
   */
  static constexpr DCMotor Andymark9015(int numMotors = 1) {
    return DCMotor(12, 0.36, 71, 3.7, rpm2radpsec(14270), numMotors);
  }

  /**
   * Returns a gearbox of Banebots RS 550 motors.
   */
  static constexpr DCMotor BanebotsRS550(int numMotors = 1) {
    return DCMotor(12, 0.38, 84, 0.4, rpm2radpsec(19000), numMotors);
  }

  /**
   * Returns a gearbox of NEO brushless motors.
   */
  static constexpr DCMotor NEO(int numMotors = 1) {
    return DCMotor(12, 2.6, 105, 1.8, rpm2radpsec(5676), numMotors);
  }

  /**
   * Returns a gearbox of NEO 550 brushless motors.
   */
  static constexpr DCMotor NEO550(int numMotors = 1) {
    return DCMotor(12, 0.97, 100, 1.4, rpm2radpsec(11000), numMotors);
  }

  /**
   * Returns a gearbox of Falcon 500 brushless motors.
   */
  static constexpr DCMotor Falcon500(int numMotors = 1) {
    return DCMotor(12, 4.69, 257, 1.5, rpm2radpsec(6380), numMotors);
  }

  /**
   * Return a gearbox of Falcon 500 motors with FOC (Field-Oriented Control)
   * enabled.
   */
  static constexpr DCMotor Falcon500FOC(int numMotors = 1) {
    // https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
    return DCMotor(12, 5.84, 304, 1.5, rpm2radpsec(6080), numMotors);
  }

  /**
   * Return a gearbox of Romi/TI_RSLK MAX motors.
   */
  static constexpr DCMotor RomiBuiltIn(int numMotors = 1) {
    // From https://www.pololu.com/product/1520/specs
    return DCMotor(4.5, 0.1765, 1.25, 0.13, rpm2radpsec(150), numMotors);
  }

  /**
   * Return a gearbox of Kraken X60 brushless motors.
   */
  static constexpr DCMotor KrakenX60(int numMotors = 1) {
    // From https://store.ctr-electronics.com/announcing-kraken-x60/
    return DCMotor(12, 7.09, 366, 2, rpm2radpsec(6000), numMotors);
  }

  /**
   * Return a gearbox of Kraken X60 brushless motors with FOC (Field-Oriented
   * Control) enabled.
   */
  static constexpr DCMotor KrakenX60FOC(int numMotors = 1) {
    // From https://store.ctr-electronics.com/announcing-kraken-x60/
    return DCMotor(12, 9.37, 483, 2, rpm2radpsec(5800), numMotors);
  }

  /**
   * Return a gearbox of Neo Vortex brushless motors.
   */
  static constexpr DCMotor NeoVortex(int numMotors = 1) {
    // From https://www.revrobotics.com/next-generation-spark-neo/
    return DCMotor(12, 3.60, 211, 3.615, rpm2radpsec(6784), numMotors);
  }

 private:
  static constexpr double rpm2radpsec(double rpm) {
    constexpr double pi = 3.141592653589793238462643383279502884L;
    return rpm * 2.0 * pi / 60.0;
  }
};

}  // namespace frc
