/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines the PIDControllerConfig class
 */

#pragma once

#include <string>

#include "../../tools/file_tool.h"
#include "../config.h"

namespace autopilot {
namespace configs {
namespace controllerconfig {

/**
 * @class PIDControllerConfig
 * @brief pid controller config
 *
 */
class PIDControllerConfig : public Config {
 public:
  /**
   * @brief Construct a new PIDControllerConfig object
   *
   */
  PIDControllerConfig() = default;

  /**
   * @brief Construct a new PIDControllerConfig object
   *
   * @param config_file_path
   */
  PIDControllerConfig(std::string& config_file_path)
      : config_file_path_(config_file_path),
        config_input_file_(config_file_path_) {}

  /**
   * @brief Construct a new PIDControllerConfig object
   *
   * @param pid_controller_config
   */
  PIDControllerConfig(PIDControllerConfig& pid_controller_config)
      : config_file_path_(pid_controller_config.GetConfigFilePath()) {}

  PIDControllerConfig& operator=(
      const PIDControllerConfig& pid_controller_config) {
    config_file_path_ = pid_controller_config.config_file_path_;
    config_input_file_.open(config_file_path_);
    ReadConfig();
    return *this;
  }

  /**
   * @brief Destroy the PIDControllerConfig object
   *
   */
  ~PIDControllerConfig() {
    if (config_input_file_) {
      config_input_file_.close();
    }
  }

  /**
   * @brief override parent class virual function
   *
   * @return int
   */
  virtual int ReadConfig();

  /**
   * @brief Get the Config File Path object
   *
   * @return std::string
   */
  const std::string& GetConfigFilePath() const { return config_file_path_; }

  /**
   * @brief Set the Config File Path object
   *
   * @param config_file_path
   */
  void SetConfigFilePath(std::string& config_file_path) {
    config_file_path_ = config_file_path;
  }

  /**
   * @brief Get the Kp object
   *
   * @return double
   */
  double GetKp() const { return kp_; }

  /**
   * @brief Set the Kp object
   *
   * @param kp
   */
  void SetKp(double kp) { kp_ = kp; }

  /**
   * @brief Get the Ki object
   *
   * @return double
   */
  double GetKi() const { return ki_; }

  /**
   * @brief Set the Ki object
   *
   * @param ki
   */
  void SetKi(double ki) { ki_ = ki; }

  /**
   * @brief Get the Kd object
   *
   * @return double
   */
  double GetKd() const { return kd_; }

  /**
   * @brief Set the Kd object
   *
   * @param kd
   */
  void SetKd(double kd) { kd_ = kd; }

  /**
   * @brief Get the Int Sat High object
   *
   * @return double
   */
  double GetIntSatHigh() const { return integrator_saturation_high_; }

  /**
   * @brief Set the Int Sat High object
   *
   * @param integrator_saturation_high
   */
  void SetIntSatHigh(double integrator_saturation_high) {
    integrator_saturation_high_ = integrator_saturation_high;
  }

  /**
   * @brief Get the Int Sat Low object
   *
   * @return double
   */
  double GetIntSatLow() const { return integrator_saturation_low_; }

  /**
   * @brief Set the Int Sat Low object
   *
   * @param integrator_saturation_low
   */
  void SetIntSatLow(double integrator_saturation_low) {
    integrator_saturation_low_ = integrator_saturation_low;
  }

  /**
   * @brief Get the Out Sat High object
   *
   * @return double
   */
  double GetOutSatHigh() const { return output_saturation_high_; }

  /**
   * @brief Set the Out Sat High object
   *
   * @param output_saturation_high
   */
  void SetOutSatHigh(double output_saturation_high) {
    output_saturation_high_ = output_saturation_high;
  }

  /**
   * @brief Get the Out Sat Law object
   *
   * @return double
   */
  double GetOutSatLow() const { return output_saturation_low_; }

  /**
   * @brief Set the Out Sat Law object
   *
   * @param output_saturation_low
   */
  void SetOutSatLow(double output_saturation_low) {
    output_saturation_low_ = output_saturation_low;
  }

 private:
  std::string config_file_path_;
  std::ifstream config_input_file_;
  autopilot::tools::FileTool file_tool_;

  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;
  double output_saturation_high_ = 0.0;
  double output_saturation_low_ = 0.0;
};

}  // namespace controllerconfig
}  // namespace configs
}  // namespace autopilot
