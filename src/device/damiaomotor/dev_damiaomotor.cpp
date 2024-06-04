#include "dev_damiaomotor.hpp"

#include "bsp_time.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX -45.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

using namespace Device;

static const uint8_t ENABLE_CMD[8] = {0XFF, 0XFF, 0XFF, 0XFF,
                                      0XFF, 0XFF, 0XFF, 0XFC};

static const uint8_t DISABLE_CMD[8] = {0XFF, 0XFF, 0XFF, 0XFF,
                                       0XFF, 0XFF, 0XFF, 0XFD};

static std::array<bool, BSP_CAN_NUM> initd = {false};

std::array<Message::Topic<Can::Pack> *, BSP_CAN_NUM> DamiaoMotor::damiao_tp_;
DamiaoMotor::DamiaoMotor(const Param &param, const char *name)
    : BaseMotor(name, param.reverse), param_(param) {
  auto rx_callback = [](Can::Pack &rx, DamiaoMotor *motor) {
    if ((rx.data[0] & 0x0f) == (motor->param_.id)) {
      motor->recv_.Overwrite(rx);
    }
    return true;
  };

  if (!initd[this->param_.can]) {
    DamiaoMotor::damiao_tp_[this->param_.can] =
        static_cast<Message::Topic<Can::Pack> *>(
            System::Memory::Malloc(sizeof(Message::Topic<Can::Pack>)));
    *DamiaoMotor::damiao_tp_[this->param_.can] = Message::Topic<Can::Pack>(
        (std::string("damiao_posspeed_can") + std::to_string(this->param_.can))
            .c_str());

    // Can::Subscribe(*DamiaoSpeedPOsition::damiao_tp_[this->param_.can],
    //              this->param_.can, 0, 1);

    Can::Subscribe(*DamiaoMotor::damiao_tp_[this->param_.can], this->param_.can,
                   this->param_.feedback_id, 1);

    initd[this->param_.can] = true;
  }

  Message::Topic<Can::Pack> motor_tp(name);

  motor_tp.RegisterCallback(rx_callback, this);

  motor_tp.Link(*this->damiao_tp_[this->param_.can]);
}

bool DamiaoMotor::Update() {
  Can::Pack pack;

  while (this->recv_.Receive(pack)) {
    this->Decode(pack);
    last_online_time_ = bsp_time_get_ms();
  }

  return true;
}

void DamiaoMotor::Decode(Can::Pack &rx) {
  if (this->param_.id != (rx.data[0] & 0x0f)) {
    return;
  }

  uint16_t raw_position = rx.data[1] << 8 | rx.data[2];

  uint16_t raw_speed = (rx.data[3] << 4) | (rx.data[4] >> 4);

  uint16_t raw_current = (rx.data[4] & 0x0f) << 8 | rx.data[5];

  float raw = uint_to_float(raw_position, P_MIN, P_MAX, 16);
  float speed = uint_to_float(raw_speed, V_MIN, V_MAX, 12);
  float current = uint_to_float(raw_current, -T_MAX, T_MAX, 12);

  this->feedback_.rotational_speed = speed;
  this->feedback_.rotor_abs_angle = raw;
  this->feedback_.torque_current = current;
}
/*摆设*/
void DamiaoMotor::Control(float output) {
  static_cast<void>(output);
  XB_ASSERT(false);
}

void DamiaoMotor::SetPosSpeed(float pos, float speed) {
  if (this->feedback_.temp > 75.0f) {
    Relax();
    OMLOG_WARNING("motor %s high temperature detected", name_);
    return;
  }
  if (reverse_) {
    pos = -pos;
  }

  Can::Pack tx_buff;

  tx_buff.index = this->param_.id + 0x100;

  uint8_t *pbuf, *vbuf;

  pbuf = (uint8_t *)&pos;
  vbuf = (uint8_t *)&speed;

  tx_buff.data[0] = *pbuf;
  tx_buff.data[1] = *(pbuf + 1);
  tx_buff.data[2] = *(pbuf + 2);
  tx_buff.data[3] = *(pbuf + 3);

  tx_buff.data[4] = *vbuf;
  tx_buff.data[5] = *(vbuf + 1);
  tx_buff.data[6] = *(vbuf + 2);
  tx_buff.data[7] = *(vbuf + 3);

  Can::SendStdPack(this->param_.can, tx_buff);
}

bool DamiaoMotor::Enable() {
  Can::Pack tx_buff;

  tx_buff.index = param_.id + 0x100;

  memcpy(tx_buff.data, ENABLE_CMD, sizeof(tx_buff.data));

  if (Can::SendStdPack(this->param_.can, tx_buff)) {
    return true;
  } else {
    return false;
  };
}
bool DamiaoMotor::Disable() {
  Can::Pack tx_buff;

  tx_buff.index = param_.id + 0x100;

  memcpy(tx_buff.data, DISABLE_CMD, sizeof(tx_buff.data));

  if (Can::SendStdPack(this->param_.can, tx_buff)) {
    return true;
  } else {
    return false;
  };
}
/*摆设*/
void DamiaoMotor::Relax() { XB_ASSERT(false); }
