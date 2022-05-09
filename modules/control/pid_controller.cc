/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "pid_controller.h"

#include <iostream>

#include "../../constant.h"

/**
 * @namespace autopilot::modules::control
 * @brief autopilot::modules::control
 *
 */
namespace autopilot {
namespace modules {
namespace control {
int PIDController::Init() {
  pid_controller_config_.ReadConfig();
  kp_ = pid_controller_config_.GetKp();
  ki_ = pid_controller_config_.GetKi();
  kd_ = pid_controller_config_.GetKd();
  integrator_saturation_high_ = pid_controller_config_.GetIntSatHigh();
  integrator_saturation_low_ = pid_controller_config_.GetIntSatLow();
  first_hit_ = true;
  integrator_hold_ = false;
  integrator_saturation_status_ = 0;

  return Constants::SUCCESS;
}

int PIDController::SetPID() {
  kp_ = pid_controller_config_.GetKp();
  ki_ = pid_controller_config_.GetKi();
  kd_ = pid_controller_config_.GetKd();

  return Constants::SUCCESS;
}

int PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;

  return Constants::SUCCESS;
}

int PIDController::ResetIntegral() {
  integral_ = 0.0;
  integrator_saturation_status_ = 0;

  return Constants::SUCCESS;
}

double PIDController::Control(double error, double dt) {
  if (dt <= 0.0) {
    std::cout << "dt <= 0, will use the last output, dt: " << dt << std::endl;
    return previous_output_;
  }

  double diff = 0.;
  double output = 0.;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }

  if (!integrator_hold_) {
    integral_ = error * dt * ki_;
    if (integral_ > integrator_saturation_high_) {
      integral_ = integrator_saturation_high_;
      integrator_saturation_status_ = 1;
    } else if (integral_ < integrator_saturation_low_) {
      integral_ = integrator_saturation_low_;
      integrator_saturation_status_ = -1;
    } else {
      integrator_saturation_status_ = 0;
    }
  }

  previous_error_ = error;
  output = error * kp_ + integral_ + diff * kd_;  // Ki already applied
  previous_output_ = output;
  return output;
}
}  // namespace control
}  // namespace modules
}  // namespace autopilot