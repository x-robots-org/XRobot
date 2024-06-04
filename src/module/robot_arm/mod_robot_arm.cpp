#include "mod_robot_arm.hpp"

#include <thread.hpp>

#include "bsp_time.h"
#include "dev_rm_motor.hpp"

using namespace Module;

RobotArm::RobotArm(Param& param, float control_freq)
    : param_(param),
      mode_(RobotArm::WORK_BOT),
      roll2_actr_(this->param_.roll2_actr, control_freq),

      yaw1_motor_(this->param_.yaw1_motor, "RobotArm_Yaw1"),
      pitch1_motor_(this->param_.pitch1_motor, "RobotArm_Pitch1"),
      pitch2_motor_(this->param_.pitch2_motor, "RobotArm_Pitch2"),
      roll1_motor_(this->param_.roll1_motor, "RobotArm_Roll1"),
      yaw2_motor_(this->param_.yaw2_motor, "RobotArm_Yaw2"),
      roll2_motor_(this->param_.roll2_motor, "RobotArm_Roll2"),
      custom_ctrl_(param.cust_ctrl),
      ctrl_lock_(true) {
  memset(&(this->cmd_), 0, sizeof(this->cmd_));

  memset(&(this->setpoint_), 0, sizeof(this->setpoint_));

  auto event_callback = [](RobotArmEvent event, RobotArm* robotarm) {
    robotarm->ctrl_lock_.Wait(UINT32_MAX);

    switch (event) {
      case SET_MODE_RELAX:
        robotarm->SetMode(RELAX);
        break;
      case SET_MODE_WORK_TOP:
        robotarm->SetMode(WORK_TOP);
        break;
      case SET_MODE_CUSTOM_CTRL:
        robotarm->SetMode(WORK_CUSTOM_CTRL);
        break;
      case SET_MODE_WORK_MID:
        robotarm->SetMode(WORK_MID);
        break;
      case SET_MODE_WORK_BOT:
        robotarm->SetMode(WORK_BOT);
        break;
      case SET_MODE_SAFE:
        robotarm->SetMode(SAFE);
        break;
      case SET_MODE_XIKUANG:
        robotarm->SetMode(XIKUANG);
        break;
      case SET_MODE_YINKUANG:
        robotarm->SetMode(YINKUANG);
        break;
      case SET_MODE_DIMIAN:
        robotarm->SetMode(DIMIAN);
        break;
      default:
        break;
    }

    robotarm->ctrl_lock_.Post();
  };

  Component::CMD::RegisterEvent<RobotArm*, RobotArmEvent>(
      event_callback, this, this->param_.EVENT_MAP);

  auto robotarm_thread = [](RobotArm* robotarm) {
    auto cmd_sub = Message::Subscriber<Component::CMD::GimbalCMD>("cmd_gimbal");

    uint32_t last_online_time = bsp_time_get_ms();

    while (1) {
      cmd_sub.DumpData(robotarm->cmd_);
      robotarm->ctrl_lock_.Wait(UINT32_MAX);
      robotarm->DamiaoSetAble();
      robotarm->UpdateFeedback();
      robotarm->Control();
      robotarm->ctrl_lock_.Post();

      robotarm->thread_.SleepUntil(2, last_online_time);
    }
  };

  this->thread_.Create(robotarm_thread, this, "robotarm_thread", 1024,
                       System::Thread::MEDIUM);
}

void RobotArm::DamiaoSetAble() {
  if (this->yaw1_able_ == 0) {
    this->yaw1_motor_.Disable();
  } else {
    this->yaw1_motor_.Enable();
  };
  if (this->pitch1_able_ == 0) {
    this->pitch1_motor_.Disable();
  } else {
    this->pitch1_motor_.Enable();
  };
  if (this->pitch2_able_ == 0) {
    this->pitch2_motor_.Disable();
  } else {
    this->pitch2_motor_.Enable();
  };
  if (this->roll1_able_ == 0) {
    this->roll1_motor_.Disable();
  } else {
    this->roll1_motor_.Enable();
  };
  if (this->yaw2_able_ == 0) {
    this->yaw2_motor_.Disable();
  } else {
    this->yaw2_motor_.Enable();
  };
}

void RobotArm::UpdateFeedback() {
  this->yaw1_motor_.Update();
  this->pitch1_motor_.Update();
  this->pitch2_motor_.Update();
  this->roll1_motor_.Update();
  this->yaw2_motor_.Update();
  this->roll2_motor_.Update();
}

void RobotArm::Control() {
  this->now_ = bsp_time_get();

  this->dt_ = TIME_DIFF(this->last_wakeup_, this->now_);

  this->last_wakeup_ = this->now_;

  float pit_cmd = 0.0f;
  float yaw_cmd = 0.0f;

  yaw_cmd = this->cmd_.eulr.yaw * this->dt_ * (0.7);
  pit_cmd = this->cmd_.eulr.pit * this->dt_ * (0.7);

  switch (this->mode_) {
    case RobotArm::WORK_BOT: {
      this->setpoint_.yaw1_theta_ += 2 * yaw_cmd;
      this->setpoint_.pitch1_theta_ += pit_cmd;

      clampf(&(this->setpoint_.yaw1_theta_), this->param_.limit.yaw1_min,
             this->param_.limit.yaw1_max);
      clampf(&(this->setpoint_.pitch1_theta_), this->param_.limit.pitch1_min,
             this->param_.limit.pitch1_max);

      this->yaw1_motor_.SetPosSpeed(this->setpoint_.yaw1_theta_, 1.6f);
      this->pitch1_motor_.SetPosSpeed(this->setpoint_.pitch1_theta_, 0.8f);
      break;
    }
    case RobotArm::WORK_MID: {
      this->setpoint_.roll1_theta_ += 2 * yaw_cmd;
      this->setpoint_.pitch2_theta_ += pit_cmd;

      clampf(&(this->setpoint_.pitch2_theta_), this->param_.limit.pitch2_min,
             this->param_.limit.pitch2_max);

      this->pitch2_motor_.SetPosSpeed(this->setpoint_.pitch2_theta_, 0.8f);
      this->roll1_motor_.SetPosSpeed(this->setpoint_.roll1_theta_, 1.6f);
      break;
    }
    case RobotArm::WORK_TOP: {
      this->setpoint_.yaw2_theta_ += pit_cmd;
      this->setpoint_roll2_ =
          Component::Type::CycleValue(this->setpoint_roll2_ + yaw_cmd);

      clampf(&(this->setpoint_.yaw2_theta_), this->param_.limit.yaw2_min,
             this->param_.limit.yaw2_max);

      this->yaw2_motor_.SetPosSpeed(this->setpoint_.yaw2_theta_, 0.8f);

      float roll2_out = this->roll2_actr_.Calculate(
          this->setpoint_roll2_, this->roll2_motor_.GetSpeed(),
          this->roll2_motor_.GetAngle(), this->dt_);
      this->roll2_motor_.Control(roll2_out);

      break;
    }
    case RobotArm::SAFE: {
      this->setpoint_.pitch1_theta_ = 0;
      this->setpoint_.pitch2_theta_ = 0;
      this->setpoint_.yaw1_theta_ = 0;
      this->setpoint_.yaw2_theta_ = 0;
      this->setpoint_.roll1_theta_ = 0;
      this->yaw1_motor_.SetPosSpeed(0, 0.6f);
      this->pitch1_motor_.SetPosSpeed(0, 0.5f);
      this->yaw2_motor_.SetPosSpeed(0, 0.6f);
      this->pitch2_motor_.SetPosSpeed(0, 0.5f);
      this->roll1_motor_.SetPosSpeed(0, 0.5f);
      break;
    }

    case RobotArm::YINKUANG: {
      this->setpoint_.pitch1_theta_ = 2.056f;
      this->setpoint_.pitch2_theta_ = -0.939f;
      this->setpoint_.yaw1_theta_ = 0;
      this->setpoint_.yaw2_theta_ = 0;
      this->setpoint_.roll1_theta_ = 0;
      this->yaw1_motor_.SetPosSpeed(0, 0.6f);
      this->pitch1_motor_.SetPosSpeed(2.056f, 0.5f);
      this->yaw2_motor_.SetPosSpeed(0, 0.6f);
      this->pitch2_motor_.SetPosSpeed(-0.939f, 0.5f);
      this->roll1_motor_.SetPosSpeed(0.0f, 0.5f);
      break;
    }

    case RobotArm::DIMIAN: {
      this->setpoint_.pitch1_theta_ = 3.426f;
      this->setpoint_.pitch2_theta_ = -1.426f;
      this->setpoint_.yaw1_theta_ = 0;
      this->setpoint_.yaw2_theta_ = 0.7546;
      this->setpoint_.roll1_theta_ = 0;
      this->yaw1_motor_.SetPosSpeed(0, 0.6f);
      this->pitch1_motor_.SetPosSpeed(3.426f, 0.5f);
      this->yaw2_motor_.SetPosSpeed(0.7546f, 0.6f);
      this->pitch2_motor_.SetPosSpeed(-1.426f, 0.5f);
      this->roll1_motor_.SetPosSpeed(0.0f, 0.5f);
      break;
    }

    case RobotArm::RELAX:
      this->yaw1_able_ = 0;
      this->pitch1_able_ = 0;
      this->pitch2_able_ = 0;
      this->roll1_able_ = 0;
      this->yaw2_able_ = 0;
      this->roll2_motor_.Relax();
      break;

    case RobotArm::XIKUANG:
      if (this->state_ == 0) {
        bsp_gpio_write_pin(BSP_GPIO_SWITCH, 0);
      } else {
        bsp_gpio_write_pin(BSP_GPIO_SWITCH, 1);
      };
      break;

    case RobotArm::WORK_CUSTOM_CTRL:
      if (custom_ctrl_.online_) {
        this->setpoint_.yaw1_theta_ = custom_ctrl_.data_.angle[0];
        this->setpoint_.pitch1_theta_ = custom_ctrl_.data_.angle[1];

        clampf(&(this->setpoint_.yaw1_theta_), this->param_.limit.yaw1_min,
               this->param_.limit.yaw1_max);
        clampf(&(this->setpoint_.pitch1_theta_), this->param_.limit.pitch1_min,
               this->param_.limit.pitch1_max);

        this->yaw1_motor_.SetPosSpeed(this->setpoint_.yaw1_theta_, 0.6f);
        this->pitch1_motor_.SetPosSpeed(this->setpoint_.pitch1_theta_, 0.5f);

        this->setpoint_.roll1_theta_ = custom_ctrl_.data_.angle[3];
        this->setpoint_.pitch2_theta_ = -custom_ctrl_.data_.angle[2];

        clampf(&(this->setpoint_.pitch2_theta_), this->param_.limit.pitch2_min,
               this->param_.limit.pitch2_max);

        this->pitch2_motor_.SetPosSpeed(this->setpoint_.pitch2_theta_, 0.5f);
        this->roll1_motor_.SetPosSpeed(this->setpoint_.roll1_theta_, 0.5f);

        this->setpoint_.yaw2_theta_ = custom_ctrl_.data_.angle[4];
        this->setpoint_roll2_ = custom_ctrl_.data_.angle[5];

        clampf(&(this->setpoint_.yaw2_theta_), this->param_.limit.yaw2_min,
               this->param_.limit.yaw2_max);

        this->yaw2_motor_.SetPosSpeed(this->setpoint_.yaw2_theta_, 0.5f);

        float roll2_out = this->roll2_actr_.Calculate(
            this->setpoint_roll2_, this->roll2_motor_.GetSpeed(),
            this->roll2_motor_.GetAngle(), this->dt_);
        this->roll2_motor_.Control(roll2_out);
      } else {
        this->yaw1_able_ = 0;
        this->pitch1_able_ = 0;
        this->pitch2_able_ = 0;
        this->roll1_able_ = 0;
        this->yaw2_able_ = 0;
        this->roll2_motor_.Relax();
      }

      break;
  }
}

void RobotArm::SetMode(RobotArm::Mode mode) {
  if (mode == this->mode_) {
    return;
  }
  if (mode == WORK_BOT) {
    this->yaw1_able_ = 1;
    this->pitch1_able_ = 1;
    this->pitch2_able_ = 1;
    this->roll1_able_ = 1;
    this->yaw2_able_ = 1;
  }
  if (mode == WORK_CUSTOM_CTRL) {
    this->yaw1_able_ = 1;
    this->pitch1_able_ = 1;
    this->pitch2_able_ = 1;
    this->roll1_able_ = 1;
    this->yaw2_able_ = 1;
  }
  if (mode == WORK_MID) {
    this->yaw1_able_ = 1;
    this->pitch1_able_ = 1;
    this->pitch2_able_ = 1;
    this->roll1_able_ = 1;
    this->yaw2_able_ = 1;
  }
  if (mode == WORK_TOP) {
    this->yaw1_able_ = 1;
    this->pitch1_able_ = 1;
    this->pitch2_able_ = 1;
    this->roll1_able_ = 1;
    this->yaw2_able_ = 1;
  }
  if (mode == XIKUANG) {
    this->state_ = !this->state_;
  }
  this->mode_ = mode;
}
