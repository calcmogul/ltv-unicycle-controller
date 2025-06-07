#include <frc/LTVUnicycleController.h>

int main() {
  frc::LTVUnicycleController controller{0.05};
  controller.Calculate({{0.0, 0.0}, 0.0}, {{0.0, 0.0}, 0.0}, 1.0, 1.0);
}
