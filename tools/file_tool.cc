/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "file_tool.h"

#include <cstdlib>
#include <string>

#include "../configs/carconfig/car_lon_config.h"
#include "../configs/controllerconfig/pid_controller_config.h"
#include "../constant.h"

namespace autopilot {
namespace tools {
int FileTool::ReadCarLonConfig(std::ifstream& config_input_file,
                               autopilot::configs::Config* config) {
  if (!config_input_file) {
    return autopilot::Constants::FAIL;
  }

  using autopilot::configs::carconfig::CarLonConfig;
  CarLonConfig* car_lon_config = dynamic_cast<CarLonConfig*>(config);
  if (nullptr == car_lon_config) {
    return autopilot::Constants::FAIL;
  }

  while (!config_input_file.eof()) {
    char buffer[512];
    config_input_file.getline(buffer, 512);
    std::string temp_string = std::string(buffer);
    int n = temp_string.size();
    int index_ = temp_string.find('=');
    if (index_ >= 0 && index_ < n) {
      continue;
    }
    int index_num = temp_string.find(':');
    std::string key = temp_string.substr(0, index_num);
    std::string value = temp_string.substr(index_num + 1);
    const char* c_str = value.c_str();
    if (key.compare("simulation step") == 0) {
      car_lon_config->SetSimStep(std::atof(value.c_str()));
    } else if (key.compare("vehicle mass") == 0) {
      car_lon_config->GetVehicleConfig().m_ = std::atof(value.c_str());
    } else if (key.compare("wheel radius") == 0) {
      car_lon_config->GetVehicleConfig().r_ = std::atof(value.c_str());
    } else if (key.compare("transmission ratio") == 0) {
      car_lon_config->GetDiverConfig().ig_ = std::atof(value.c_str());
    } else if (key.compare("final driver ratio") == 0) {
      car_lon_config->GetDiverConfig().i0_ = std::atof(value.c_str());
    } else if (key.compare("mechanical efficiency") == 0) {
      car_lon_config->GetDiverConfig().eta_ = std::atof(value.c_str());
    } else if (key.compare("a0") == 0) {
      car_lon_config->GetDiverConfig().a0_ = std::atof(value.c_str());
    } else if (key.compare("a1") == 0) {
      car_lon_config->GetDiverConfig().a1_ = std::atof(value.c_str());
    } else if (key.compare("a2") == 0) {
      car_lon_config->GetDiverConfig().a2_ = std::atof(value.c_str());
    } else if (key.compare("rolling resistance ratio") == 0) {
      car_lon_config->GetRollResistConfig().f_ = std::atof(value.c_str());
    } else if (key.compare("air resistance coefficient") == 0) {
      car_lon_config->GetAirResistConfig().Cd_ = std::atof(value.c_str());
    } else if (key.compare("air density") == 0) {
      car_lon_config->GetAirResistConfig().rho_ = std::atof(value.c_str());
    } else if (key.compare("windward aera") == 0) {
      car_lon_config->GetAirResistConfig().A_ = std::atof(value.c_str());
    } else if (key.compare("slope angle") == 0) {
      car_lon_config->GetSlopeResistConfig().alpha_ = std::atof(value.c_str());
    } else if (key.compare("conversion coefficient of vehicle rotating mass") ==
               0) {
      car_lon_config->GetAccelerationResistConfig().delte_ =
          std::atof(value.c_str());
    } else if (key.compare("braking coefficient") == 0) {
      car_lon_config->GetBrakeConfig().kb_ = std::atof(value.c_str());
    }
  }
  return autopilot::Constants::SUCCESS;
}

int FileTool::ReadPIDControllerConfig(std::ifstream& config_input_file,
                                      autopilot::configs::Config* config) {
  if (!config_input_file) {
    return autopilot::Constants::FAIL;
  }

  using autopilot::configs::controllerconfig::PIDControllerConfig;
  PIDControllerConfig* pid_controller_config =
      dynamic_cast<PIDControllerConfig*>(config);
  if (nullptr == pid_controller_config) {
    return autopilot::Constants::FAIL;
  }

  while (!config_input_file.eof()) {
    char buffer[512];
    config_input_file.getline(buffer, 512);
    std::string temp_string = std::string(buffer);
    int n = temp_string.size();
    int index_ = temp_string.find('=');
    if (index_ >= 0 && index_ < n) {
      continue;
    }
    int index_num = temp_string.find(':');
    std::string key = temp_string.substr(0, index_num);
    std::string value = temp_string.substr(index_num + 1);
    const char* c_str = value.c_str();
    if (key.compare("kp") == 0) {
      pid_controller_config->SetKp(std::atof(value.c_str()));
    } else if (key.compare("ki") == 0) {
      pid_controller_config->SetKi(std::atof(value.c_str()));
    } else if (key.compare("kd") == 0) {
      pid_controller_config->SetKd(std::atof(value.c_str()));
    } else if (key.compare("integrator_saturation_high") == 0) {
      pid_controller_config->SetIntSatHigh(std::atof(value.c_str()));
    } else if (key.compare("integrator_saturation_low") == 0) {
      pid_controller_config->SetIntSatLow(std::atof(value.c_str()));
    } else if (key.compare("output_saturation_high") == 0) {
      pid_controller_config->SetOutSatHigh(std::atof(value.c_str()));
    } else if (key.compare("output_saturation_low") == 0) {
      pid_controller_config->SetOutSatLow(std::atof(value.c_str()));
    }
  }
  return autopilot::Constants::SUCCESS;
}
}  // namespace tools
}  // namespace autopilot