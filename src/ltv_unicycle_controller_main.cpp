#include <frc/LTVUnicycleController.h>

int main() {
  constexpr double dt = 0.05;

  frc::LTVUnicycleController unicycle_controller{dt};
  unicycle_controller.Calculate({{0.0, 0.0}, 0.0}, {{0.0, 0.0}, 0.0}, 1.0, 1.0);
}
