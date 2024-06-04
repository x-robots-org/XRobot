#pragma once

#include <device.hpp>

#include "dev_can.hpp"
#include "dev_motor.hpp"
namespace Device {
class DamiaoMotor : public BaseMotor {
 public:
  typedef struct {
    uint32_t id; /*设置CAN ID*/ /*控制帧ID为CAN ID偏移0x100*/
    bsp_can_t can;
    uint32_t feedback_id;
    bool reverse;
  } Param;

  DamiaoMotor(const Param &param, const char *name);
  void Control(float output);
  bool Update();
  void Relax();
  void Decode(Can::Pack &rx);
  void SetPosSpeed(float pos, float speed);
  bool Enable();
  bool Disable();

 private:
  Param param_;

  System::Queue<Can::Pack> recv_ = System::Queue<Can::Pack>(1);

  static std::array<Message::Topic<Can::Pack> *, BSP_CAN_NUM> damiao_tp_;
};
}  // namespace Device
