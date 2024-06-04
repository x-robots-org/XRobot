#include <comp_cmd.hpp>
#include <vector>

#include "bsp_gpio.h"
#include "comp_actuator.hpp"
#include "comp_cmd.hpp"
#include "comp_pid.hpp"
#include "dev_damiaomotor.hpp"
#include "dev_motor.hpp"
// #include "dev_referee.hpp"
#include "dev_custom_controller.hpp"
#include "dev_rm_motor.hpp"
#include "module.hpp"

namespace Module {
class RobotArm {
 public:
  /*机械臂运行模式*/
  typedef enum {
    RELAX, /*放松，电机不输出*/
    WORK_TOP,
    WORK_MID,
    WORK_BOT,
    WORK_CUSTOM_CTRL,
    SAFE,
    XIKUANG,
    YINKUANG,
    DIMIAN
  } Mode;

  typedef enum {
    SET_MODE_RELAX,
    SET_MODE_WORK_TOP,
    SET_MODE_WORK_MID,
    SET_MODE_WORK_BOT,
    SET_MODE_SAFE,
    SET_MODE_CUSTOM_CTRL,
    SET_MODE_XIKUANG,
    SET_MODE_YINKUANG,
    SET_MODE_DIMIAN
  } RobotArmEvent;

  typedef struct Param {
    const std::vector<Component::CMD::EventMapItem> EVENT_MAP;

    Component::PosActuator::Param roll2_actr;

    Device::DamiaoMotor::Param yaw1_motor;
    Device::DamiaoMotor::Param pitch1_motor;
    Device::DamiaoMotor::Param pitch2_motor;
    Device::DamiaoMotor::Param roll1_motor;
    Device::DamiaoMotor::Param yaw2_motor;
    Device::RMMotor::Param roll2_motor;

    struct {
      float yaw1_max; /*大yaw,180度活动范围*/
      float yaw1_min;
      float pitch1_max; /*pitch1,0,85*/
      float pitch1_min;
      float pitch2_max; /*pitch2，-270,0*/
      float pitch2_min;
      float yaw2_max; /*小yaw,-90,+90*/
      float yaw2_min;
    } limit;

    Device::CustomController::Param cust_ctrl;
  } Param;

  RobotArm(Param &param, float control_freq);

  void UpdateFeedback();

  void Control();

  void DamiaoSetAble();

  void SetMode(Mode mode);

 private:
  Param param_;

  float dt_ = 0.0f;

  uint64_t last_wakeup_ = 0;

  uint64_t now_ = 0;

  Mode mode_ = WORK_BOT;

  Component::PosActuator roll2_actr_;

  Device::DamiaoMotor yaw1_motor_;
  Device::DamiaoMotor pitch1_motor_;
  Device::DamiaoMotor pitch2_motor_;
  Device::DamiaoMotor roll1_motor_;
  Device::DamiaoMotor yaw2_motor_;
  Device::RMMotor roll2_motor_;

  Device::CustomController custom_ctrl_;

  bool yaw1_able_ = 0;
  bool pitch1_able_ = 0;
  bool pitch2_able_ = 0;
  bool roll1_able_ = 0;
  bool yaw2_able_ = 0;

  System::Thread thread_;

  System::Semaphore ctrl_lock_;

  struct {
    float yaw1_theta_ = 0;
    float pitch1_theta_ = 0;
    float pitch2_theta_ = 0;
    float roll1_theta_ = 0;
    float yaw2_theta_ = 0;
  } setpoint_; /*用于接收控制器发来的各关节角度*/

  float setpoint_roll2_;

  bool state_ = 0;

  Component::CMD::GimbalCMD cmd_;
};

}  // namespace Module
