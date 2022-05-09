/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines the PIDController class
 */

#pragma once

#include "../../configs/controllerconfig/pid_controller_config.h"
#include "controller.h"

/**
 * @namespace autopilot::modules::control
 * @brief autopilot::modules::control
 *
 */
namespace autopilot {
namespace modules {
namespace control {

/**
 * @class PIDController
 * @brief A proportional-integral-derivative controller for speed using defualt
 * integral hold
 *
 */
class PIDController : public Controller {
 public:
  PIDController(std::string& config_file_path)
      : pid_controller_config_(config_file_path) {}

  ~PIDController() = default;

  /**
   * @brief Init the pid controller
   *
   * @return int
   */
  int Init();

  /**
   * @brief Set pid controller config
   *
   * @param pid_controller_config
   * @return int
   */
  int SetPIDControllerConfig(
      autopilot::configs::controllerconfig::PIDControllerConfig&
          pid_controller_config) {
    pid_controller_config_ = pid_controller_config;
  }

  /**
   * @brief Set pid config
   *
   * @return int
   */
  int SetPID();

  /**
   * @brief Reset pid controller
   *
   * @return int
   */
  int Reset();

  /**
   * @brief Reset the integral
   *
   * @return int
   */
  int ResetIntegral();

  /**
   * @brief Set the Integral Hold object
   *
   * @param integrator_hold
   */
  void SetIntegralHold(bool integrator_hold) {
    integrator_hold_ = integrator_hold;
  }

  /**
   * @brief Get the Integral Hold object
   *
   * @return int
   */
  bool GetIntegralHold() const { return integrator_hold_; }

  /**
   * @brief Get the Integral Saturation Status object
   *
   * @return int
   */
  int GetIntegralSaturationStatus() const {
    return integrator_saturation_status_;
  }

  /**
   * @brief compute control value based on the error using pid control algorithm
   *
   * @param error input error
   * @param dt sample cycle / control cycle
   * @return control value based on PID terms
   */
  double Control(double error, double dt);

 protected:
  autopilot::configs::controllerconfig::PIDControllerConfig
      pid_controller_config_;

  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_hold_ = false;
  int integrator_saturation_status_ = 0;
  double output_saturation_high_ = 0.0;
  double output_saturation_low_ = 0.0;
};

}  // namespace control
}  // namespace modules
}  // namespace autopilot
