#include <frc/LTVDifferentialDriveController.h>
#include <frc/IdentityDrivetrainSystem.h>

int main() {
  constexpr double dt = 0.05;
  constexpr double trackwidth = 0.9;

  constexpr double linear_kv = 3.02;      // V/(m/s)
  constexpr double linear_ka = 0.642;     // V/(m/s²)
  constexpr double angular_kv = 1.382;    // V/(m/s)
  constexpr double angular_ka = 0.08495;  // V/(m/s²)
  auto system = frc::IdentifyDrivetrainSystem(linear_kv, linear_ka, angular_kv,
                                              angular_ka);

  frc::LTVDifferentialDriveController diff_drive_controller{
      system, trackwidth, {0.0625, 0.125, 2.5, 0.95, 0.95}, {12.0, 12.0}, dt};
  diff_drive_controller.Calculate({{0.0, 0.0}, 0.0}, 1.0, 1.0,
                                  {{0.0, 0.0}, 0.0}, 1.0, 1.0);
}
