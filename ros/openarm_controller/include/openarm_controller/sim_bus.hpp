// Simulated arm for running the controller without hardware: rigid-body dynamics of one arm from the ct_models
// OpenArm model (M, C, G, generated code), the Damiao MIT law with the openarm_can encoding ranges (quantized
// commands and feedback), joint friction, and fault injection for tests.
#pragma once

#include "openarm_controller/arm_bus.hpp"

#include <memory>
#include <string>

namespace openarm_controller
{

struct SimBusOptions
{
  std::string arm = "right";
  Vec7 q0 = (Vec7() << 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0).finished();  // start pose (hanging)
  double cycle_s = 0.001;           // simulated time per refresh() / command() call
  // Integration step. The regularized Coulomb term has slope coulomb / 0.01 [N m s/rad]; with the wrist inertia
  // (~1e-3 kg m^2) explicit integration needs <= 0.1 ms to stay stable.
  double step_s = 0.0001;
  Vec7 coulomb_nm = Vec7::Zero();   // per joint, coulomb tanh(dq / 0.01)
  Vec7 viscous_nms = Vec7::Zero();
  // The real arm does not move while its motors are disabled (gearbox friction): the plant is frozen then.
  bool hold_when_disabled = true;
  double stale_after_s = -1.0;      // >= 0: replies stop after this simulated time
  int fault_joint = -1;             // >= 0: this motor reports fault code 0x9 after fault_after_s
  double fault_after_s = -1.0;
};

class SimBus : public ArmBus
{
public:
  explicit SimBus(SimBusOptions options);
  virtual ~SimBus() override;
  void enable() override;
  void disable() override;
  void refresh() override;
  void command(const MotorCommand & command) override;
  ArmState read() override;
  double now() const override;
  bool realtime() const override;

  int enableCalls() const;
  bool enabled() const;
  // True joint position of the plant (not quantized).
  Vec7 trueQ() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace openarm_controller
