#include "robot.hpp"

#include <system.hpp>
#include <thread.hpp>

#include "dev_rm_motor.hpp"
// #include "mod_engineer_chassis.hpp"

using namespace Robot;

/* clang-format off */
Robot::ArmEngineer::Param param = {
  .blink = {
    .gpio = BSP_GPIO_LED,
    .timeout = 200,
    },

.chassis = {
      .type = Component::Mixer::MECANUM,


      .EVENT_MAP = {
        Component::CMD::EventMapItem{
          Component::CMD::CMD_EVENT_LOST_CTRL,
          Module::RMChassis::SET_MODE_RELAX
        },
        Component::CMD::EventMapItem{
          Device::DR16::DR16_SW_L_POS_TOP,
          Module::RMChassis::SET_MODE_RELAX
        },
        Component::CMD::EventMapItem{
          Device::DR16::DR16_SW_L_POS_MID,
          Module::RMChassis::SET_MODE_INDENPENDENT
        },
      },

      .actuator_param = {
        Component::SpeedActuator::Param{
          .speed = {
            .k = 0.0001f,
            .p = 1.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .cycle = false,
          },

          .in_cutoff_freq = -1.0f,

          .out_cutoff_freq = -1.0f,

        },
        Component::SpeedActuator::Param{
          .speed = {
            .k = 0.0001f,
            .p = 1.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .cycle = false,
          },

          .in_cutoff_freq = -1.0f,

          .out_cutoff_freq = -1.0f,
        },
        Component::SpeedActuator::Param{
          .speed = {
            .k = 0.0001f,
            .p = 1.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .cycle = false,
          },

          .in_cutoff_freq = -1.0f,

          .out_cutoff_freq = -1.0f,
        },
        Component::SpeedActuator::Param{
          .speed = {
            .k = 0.0001f,
            .p = 1.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .cycle = false,
          },

          .in_cutoff_freq = -1.0f,

          .out_cutoff_freq = -1.0f,
        },
      },

      .motor_param = {
        Device::RMMotor::Param{
            .id_feedback = 0x201,
            .id_control = M3508_M2006_CTRL_ID_BASE,
            .model = Device::RMMotor::MOTOR_M3508,
            .can = BSP_CAN_3,
        },
        Device::RMMotor::Param{
            .id_feedback = 0x202,
            .id_control = M3508_M2006_CTRL_ID_BASE,
            .model = Device::RMMotor::MOTOR_M3508,
            .can = BSP_CAN_3,
        },
        Device::RMMotor::Param{
            .id_feedback = 0x203,
            .id_control = M3508_M2006_CTRL_ID_BASE,
            .model = Device::RMMotor::MOTOR_M3508,
            .can = BSP_CAN_3,
        },
        Device::RMMotor::Param{
            .id_feedback = 0x204,
            .id_control = M3508_M2006_CTRL_ID_BASE,
            .model = Device::RMMotor::MOTOR_M3508,
            .can = BSP_CAN_3,
        },
    },
  },

  .robotarm = {
  .EVENT_MAP = {
        Component::CMD::EventMapItem{
          Component::CMD::CMD_EVENT_LOST_CTRL,
          Module::RobotArm::SET_MODE_RELAX
        },
        Component::CMD::EventMapItem{
          Device::DR16::DR16_SW_R_POS_TOP,
          Module::RobotArm::SET_MODE_WORK_TOP
        },
        Component::CMD::EventMapItem{
          Device::DR16::DR16_SW_R_POS_MID,
         Module::RobotArm::SET_MODE_WORK_MID
        },
        Component::CMD::EventMapItem{
          Device::DR16::DR16_SW_R_POS_BOT,
          Module::RobotArm::SET_MODE_WORK_BOT
        },
        Component::CMD::EventMapItem{
          Device::DR16::DR16_SW_L_POS_BOT,
          Module::RobotArm::SET_MODE_XIKUANG
        },
        Component::CMD::EventMapItem{
        Device::DR16::KEY_R,
        Module::RobotArm::SET_MODE_YINKUANG
      },
      Component::CMD::EventMapItem{
        Device::DR16::KEY_Z,
        Module::RobotArm::SET_MODE_SAFE
      },
            Component::CMD::EventMapItem{
        Device::DR16::KEY_C,
        Module::RobotArm::SET_MODE_DIMIAN
      },
  },
  .roll2_actr={
.speed = {
          .k = 0.1f,
          .p = 0.025f,
          .i = 0.2f,
          .d = 0.0f,
          .i_limit = 0.2f,
          .out_limit = 1.0f,
          .d_cutoff_freq = -1.0f,
          .cycle = false,
        },

        .position = {
          .k = 40.0f,
          .p = 6.1f,
          .i = 1.0f,
          .d = 0.4f,
          .i_limit = 2.0f,
          .out_limit = 200.0f,
          .d_cutoff_freq = -1.0f,
          .cycle = true,
        },

        .in_cutoff_freq = -1.0f,

        .out_cutoff_freq = -1.0f,
  },

.yaw1_motor={
.id = 01,
.can = BSP_CAN_2,
.feedback_id = 0,
.reverse = 0,
},
.pitch1_motor={
 .id = 02,
.can = BSP_CAN_2,
.feedback_id = 1,
.reverse = 0,
},
.pitch2_motor={
  .id = 03,
.can = BSP_CAN_1,
.feedback_id = 2,
.reverse = 0,
},
.roll1_motor={
  .id = 04,
.can = BSP_CAN_1,
.feedback_id = 3,
.reverse = 0,
},
.yaw2_motor={
  .id = 05,
.can = BSP_CAN_1,
.feedback_id = 4,
.reverse = 1,
},

.roll2_motor={
        .id_feedback = 0x208,
        .id_control = GM6020_CTRL_ID_BASE,
        .model = Device::RMMotor::MOTOR_GM6020,
        .can = BSP_CAN_1,
},

.limit = {
        .yaw1_max = 0.0f,
        .yaw1_min =-7.5f,
        .pitch1_max = 3.5f,
        .pitch1_min = 0.0f,
        .pitch2_max = 0.0f,
        .pitch2_min = -4.7f,
        .yaw2_max = 1.57f,
        .yaw2_min = -1.57f,
      },

// mt6701的范围为0-2PI，先修改自定义控制器六个轴的零点，保证在电机内置编码器反馈值最小的时候该轴的mt6701的值接近0（保证不会突变到6.28）
// 再修改下面的offset，例如yaw_1轴电机内置编码器的范围为-6.28到0，该轴的mt6701的值为0.1，则offset为-6.28-0.1

.cust_ctrl = {
  .offset = {
    -6.28f,
    -1+0.1f,
    -0.10f,
    -0.20+0.1f,
    -3.82+0.1f,
    -4.29f
  }
}
},


};
/* clang-format on */

void robot_init() {
  System::Start<Robot::ArmEngineer, Robot::ArmEngineer::Param>(param, 500.0f);
}
