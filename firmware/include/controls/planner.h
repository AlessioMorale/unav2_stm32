#include <math.h>
#include <stdint.h>
namespace unav::controls {

template <uint32_t axes> class Planner {

public:
  Planner() {
  }
  void setLimits(float velocity, float acceleration) {
    limits.vMax = velocity;
    limits.aMax = acceleration;
  }

  void update(float (&velocitySetpoints)[axes], float measuredVelocity[axes], float dt) {
    const float dVMax = limits.aMax * dt;

    for (int i = 0; i < axes; i++) {
      // enforce Velocity limit
      if (fabsf(velocitySetpoints[i]) > limits.vMax) {
        velocitySetpoints[i] = copysignf(limits.vMax, velocitySetpoints[i]);
      }

      // enforce Accel limits
      const float dV = velocitySetpoints[i] - measuredVelocity[i];
      if (fabsf(dV) > dVMax) {
        velocitySetpoints[i] = measuredVelocity[i] + copysignf(dVMax, dV);
      }
    }
  }

  ~Planner() {
  }

private:
  typedef struct {
    float vMax{0.0f};
    float aMax{0.0f};
  } limits_t;
  typedef struct {
    float lastVelocity{0.0f};
  } AxisStatus_t;

  limits_t limits{};
};
} // namespace unav::controls