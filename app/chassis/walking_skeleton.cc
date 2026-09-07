#include "app/chassis/walking_skeleton.h"

#include "app/chassis/kinematics.h"
#include "app/chassis/safety_gate.h"

namespace robotcar01::chassis {

WheelSpeedRequest RunMotionSliceOnce(const RobotCarConfig& config,
                                     const MotionCommand& command,
                                     const SafetyStatus& safety) {
  const WheelTarget target = ComputeWheelTarget(config, command);
  return RunSafetyGate(target, safety);
}

}  // namespace robotcar01::chassis
