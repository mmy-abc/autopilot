/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines the CarLonModel class
 */

#pragma once

#include <vector>

#include "../../configs/carconfig/car_lon_config.h"
#include "../../constant.h"

/**
 * @namespace autopilot::models::carmodel
 * @brief autopilot::models::carmodel
 */
namespace autopilot {
namespace models {
namespace carmodel {
/**
 * @class CarLonModel
 *
 * @brief General longitudinal dynamics model of automobile
 */
class CarLonModel {
 public:
  // /**
  //  * @brief Construct a new Car Lon Model object
  //  *
  //  */
  // CarLonModel() = default;

  /**
   * @brief Construct a new Car Lon Model object
   *
   * @param config_file_path
   */
  CarLonModel(std::string& config_file_path)
      : car_lon_config_(config_file_path) {}

  /**
   * @brief Destroy the Car Lon Model object
   *
   */
  ~CarLonModel() = default;

  /**
   * @brief Init longitudinal dynamics model
   *
   * @param config_file_path
   * @return int
   */
  int Init();

  /**
   * @brief Get the Position object
   *
   * @return double
   */
  double GetPosition() { return position_; }

  /**
   * @brief Set the Position object
   *
   * @param position
   */
  void setPosition(double position) { position_ = position; }

  /**
   * @brief Get the Velocity object
   *
   * @return double
   */
  double GetVelocity() { return velocity_; }

  /**
   * @brief Set the Velocity object
   *
   * @param velocity
   */
  void SetVelocity(double velocity) { velocity_ = velocity; }

  /**
   * @brief Get the Acceleration object
   *
   * @return double
   */
  double GetAcceleration() { return acceleration_; }

  /**
   * @brief Set the Acceleration object
   *
   * @param acceleration
   */
  void SetAcceleration(double acceleration) { acceleration_ = acceleration; }

  /**
   * @brief Get the Car Gear object
   *
   * @return autopilot::CarGear
   */
  autopilot::CarGear GetCarGear() { return gear_; }

  void SetCarGear(autopilot::CarGear gear) { gear_ = gear; }

  /**
   * @brief Get the Throttle object
   *
   * @return double
   */
  double GetThrottle() { return throttle_; }

  /**
   * @brief Set the Throttle object
   *
   * @param throttle
   */
  void SetThrottle(double throttle) { throttle_ = throttle; }

  /**
   * @brief Get the Brake object
   *
   * @return double
   */
  double GetBrake() { return brake_; }

  /**
   * @brief Set the Brake object
   *
   * @param brake
   */
  void SetBrake(double brake) { brake_ = brake; }

  /**
   * @brief Get the Time object
   *
   * @return double
   */
  double GetTime() { return time_; }

  /**
   * @brief Set the Time object
   *
   * @param time
   */
  void SetTime(double time) { time_ = time; }

  /**
   * @brief Get the Car Lon Config object
   *
   * @return autopilot::configs::carconfig::CarLonConfig&
   */
  autopilot::configs::carconfig::CarLonConfig& GetCarLonConfig() {
    return car_lon_config_;
  }

  /**
   * @brief Set the Car Lon Config object
   *
   * @param car_lon_config
   */
  void SetCarLonConfig(
      autopilot::configs::carconfig::CarLonConfig& car_lon_config) {
    car_lon_config_ = car_lon_config;
  }

  /**
   * @brief Set state vector
   *
   * @return int
   */
  int SetX();

  /**
   * @brief derivative of state
   *
   * @return int
   */
  int SetDX();

  /**
   * @brief Reset states
   *
   * @return int
   */
  int ReSetStates();

  /**
   * @brief Update state vector
   *
   * @return int
   */
  int Update();

  /**
   * @brief Advance one simulation step by adopting Runge-Kutta fourth-order
   * algorithm
   *
   * @return int
   */
  int Step();

 private:
  autopilot::configs::carconfig::CarLonConfig car_lon_config_;

  double position_;  // position of automobile (state)

  double velocity_;  // velocity of automobile (state)

  double acceleration_;  // acceleration of automobile (state)

  double prev_velocity_;  // prev velocity of automobile (state)

  double time_;  // run time;

  double throttle_;  // throttle of automobile (input)

  double brake_;  // brake of automobile (input)

  autopilot::CarGear gear_;  // gear of automobile (input)

  double
      x_all_[autopilot::Constants::CAR_LON_X_DIM];  // state vector: position_
                                                    // and velocity_ (
                                                    // intermediate state)

  double d_x_all_[autopilot::Constants::CAR_LON_X_DIM];  // derivative of state
                                                         // vector (
                                                         // intermediate state)

 public:
  std::vector<double> positions_;  // position set

  std::vector<double> velocitys_;  // velocity set

  std::vector<double> accelerations_;  // acceleration set

  std::vector<double> times_;  // time set
};
}  // namespace carmodel
}  // namespace models
}  // namespace autopilot