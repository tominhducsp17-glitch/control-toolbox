// The real arm through openarm_can 1.4.0 (CAN-FD, MIT mode). Only the 7 arm motors are used; the gripper motor
// is never touched.
#pragma once

#include "openarm_controller/arm_bus.hpp"

#include <memory>
#include <string>

namespace openarm_controller
{

class OpenArmCanBus : public ArmBus
{
public:
  // recv_timeout_us: wait for the first reply after each query or command (openarm_can recv_all).
  explicit OpenArmCanBus(const std::string & can_interface, int recv_timeout_us = 1000);
  virtual ~OpenArmCanBus() override;
  void enable() override;
  void disable() override;
  void refresh() override;
  void command(const MotorCommand & command) override;
  ArmState read() override;
  double now() const override;
  bool realtime() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace openarm_controller
