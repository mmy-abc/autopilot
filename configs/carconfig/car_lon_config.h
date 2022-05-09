/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines the CarLonConfig class
 */

#pragma once

#include <fstream>
#include <string>

#include "../../tools/file_tool.h"
#include "../config.h"

/**
 * @namespace autopilot::configs::carconfig
 * @brief autopilot::configs::carconfig
 */
namespace autopilot {
namespace configs {
namespace carconfig {
/**
 * @class CarLonConfig
 * @brief Configurations of longitudinal dynamics model of automobile
 */
class CarLonConfig : public Config {
 public:
  /**
   * @brief Construct a new Car Lon Config object
   */
  CarLonConfig() = default;

  /**
   * @brief Construct a new Car Lon Config object
   *
   * @param configfilepath
   */
  CarLonConfig(std::string& config_file_path)
      : config_file_path_(config_file_path),
        config_input_file_(config_file_path_) {}

  CarLonConfig(CarLonConfig& car_lon_config)
      : CarLonConfig(car_lon_config.GetConfigFilePath()) {
    ReadConfig();
  }

  CarLonConfig& operator=(const CarLonConfig& car_lon_config) {
    config_file_path_ = car_lon_config.config_file_path_;
    config_input_file_.open(config_file_path_);
    ReadConfig();
    return *this;
  }

  /**
   * @brief Destroy the Car Lon Config object
   */
  virtual ~CarLonConfig() {
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

 private:
  std::string config_file_path_;

  std::ifstream config_input_file_;

  autopilot::tools::FileTool file_tool_;

  double sim_step_ = 0.0;  // simulation step

  // vehicle config
  struct VehicleConfig {
    double m_ = 0.0;  // vehicle mass
    double r_ = 0.0;  // wheel radius
  } vehicle_config_;

  // diver config
  struct DiverConfig {
    double ig_ = 0.0;   // transmission ratio
    double i0_ = 0.0;   // final driver ratio
    double eta_ = 0.0;  // mechanical efficiency
    double a0_ = 0.0;   // constant term coefficient of torque formula
    double a1_ = 0.0;   // first order coefficient of torque formula
    double a2_ = 0.0;   // quadratic term coefficient of torque formula
  } diver_config_;

  // rolling resistance config
  struct RollResistConfig {
    double f_ = 0.0;  // rolling resistance ratio
  } roll_resist_config_;

  // air resistance config
  struct AirResistConfig {
    double Cd_ = 0.0;   // air resistance coefficient
    double rho_ = 0.0;  // air density
    double A_ = 0.0;    // windward aera
  } air_resist_config_;

  // slope resistance config
  struct SlopeResistConfig {
    double alpha_ = 0.0;  // slope angle
  } slope_resist_config_;

  // acceleration resistance config
  struct AccelerationResistConfig {
    double delte_ = 0.0;  // conversion coefficient of vehicle rotating mass
  } acceleration_resist_config_;

  // braking config
  struct BrakeConfig {
    double kb_ = 0.0;  // braking coefficient
  } brake_config_;

 public:
  /**
   * @brief Get the Sim Step object
   *
   * @return double
   */
  double GetSimStep() { return sim_step_; }

  void SetSimStep(double sim_step) { sim_step_ = sim_step; }

  /**
   * @brief Get the Vehicle Config object
   *
   * @return VehicleConfig&
   */
  VehicleConfig& GetVehicleConfig() { return vehicle_config_; }

  /**
   * @brief Set the Vehicle Config object
   *
   * @param vehicle_config
   */
  void SetVehicleConfig(VehicleConfig& vehicle_config) {
    vehicle_config_ = vehicle_config;
  }

  /**
   * @brief Get the Diver Config object
   *
   * @return DiverConfig&
   */
  DiverConfig& GetDiverConfig() { return diver_config_; }

  /**
   * @brief Set the Diver Config object
   *
   * @param diver_config
   */
  void SetDiverConfig(DiverConfig& diver_config) {
    diver_config_ = diver_config;
  }

  /**
   * @brief Get the Roll Resist Config object
   *
   * @return RollResistConfig&
   */
  RollResistConfig& GetRollResistConfig() { return roll_resist_config_; }

  /**
   * @brief Set the Roll Resist Config object
   *
   * @param roll_resist_config
   */
  void SetRollResistConfig(RollResistConfig& roll_resist_config) {
    roll_resist_config_ = roll_resist_config;
  }

  /**
   * @brief Get the Air Resist Config object
   *
   * @return AirResistConfig&
   */
  AirResistConfig& GetAirResistConfig() { return air_resist_config_; }

  /**
   * @brief Set the Air Resist Config object
   *
   * @param air_resist_config
   */
  void SetAirResistConfig(AirResistConfig& air_resist_config) {
    air_resist_config_ = air_resist_config;
  }

  /**
   * @brief Get the Slope Resist Config object
   *
   * @return SlopeResistConfig&
   */
  SlopeResistConfig& GetSlopeResistConfig() { return slope_resist_config_; }

  /**
   * @brief Set the Slope Resist Config object
   *
   * @param slope_resist_config
   */
  void SetSlopeResistConfig(SlopeResistConfig& slope_resist_config) {
    slope_resist_config_ = slope_resist_config;
  }

  /**
   * @brief Get the Acceleration Resist Config object
   *
   * @return AccelerationResistConfig&
   */
  AccelerationResistConfig& GetAccelerationResistConfig() {
    return acceleration_resist_config_;
  }

  /**
   * @brief Set the Acceleration Resist Config object
   *
   * @param acceleration_resist_config
   */
  void SetAccelerationResistConfig(
      AccelerationResistConfig& acceleration_resist_config) {
    acceleration_resist_config_ = acceleration_resist_config;
  }

  /**
   * @brief Get the Brake Config object
   *
   * @return BrakeConfig&
   */
  BrakeConfig& GetBrakeConfig() { return brake_config_; }

  /**
   * @brief Set the Brake Config object
   *
   * @param brake_config
   */
  void SetBrakeConfig(BrakeConfig& brake_config) {
    brake_config_ = brake_config;
  }

  /**
   * @brief Get the Config File Path object
   *
   * @return std::string
   */
  std::string& GetConfigFilePath() { return config_file_path_; }

  /**
   * @brief Set the Config File Path object
   *
   * @param config_file_path
   */
  void SetConfigFilePath(std::string& config_file_path) {
    config_file_path_ = config_file_path;
  }
};
}  // namespace carconfig
}  // namespace configs
}  // namespace autopilot