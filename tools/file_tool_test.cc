/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "file_tool.h"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "../configs/carconfig/car_lon_config.h"
#include "../configs/controllerconfig/pid_controller_config.h"

namespace autopilot {
namespace tools {
class FileToolTest : public ::testing::Test {
 public:
  void SetUp() {
    std::string config_file_path =
        "/home/ymm/mycode/autopilot/data/carlonconfiginput.txt";
    std::ifstream config_input_file(config_file_path.c_str(), std::ios::in);
    // ASSERT_NE(config_input_file, 0);
    car_config_input_file_ = std::move(config_input_file);
    config_file_path = "/home/ymm/mycode/autopilot/data/pidcontrollerinput.txt";
    std::ifstream pid_config_input_file(config_file_path);
    pid_config_input_file_ = std::move(pid_config_input_file);
  }
  void TearDown() {
    if (car_config_input_file_) {
      car_config_input_file_.close();
    }
    if (pid_config_input_file_) {
      pid_config_input_file_.close();
    }
  }

 protected:
  autopilot::tools::FileTool file_tool_;
  autopilot::configs::carconfig::CarLonConfig car_lon_config_;
  autopilot::configs::controllerconfig::PIDControllerConfig
      pid_controller_config_;
  std::ifstream car_config_input_file_;
  std::ifstream pid_config_input_file_;
};

TEST_F(FileToolTest, ReadCarLonConfig) {
  file_tool_.ReadCarLonConfig(car_config_input_file_, &car_lon_config_);
  double ep = 1e-6;
  EXPECT_GE(ep, fabs(car_lon_config_.GetSimStep() - 0.005));
  EXPECT_GE(ep, fabs(car_lon_config_.GetVehicleConfig().m_ - 2000.0));
  EXPECT_GE(ep, fabs(car_lon_config_.GetVehicleConfig().r_ - 0.3));
  EXPECT_GE(ep, fabs(car_lon_config_.GetDiverConfig().ig_ - 3.0));
  EXPECT_GE(ep, fabs(car_lon_config_.GetDiverConfig().i0_ - 4.0));
  EXPECT_GE(ep, fabs(car_lon_config_.GetDiverConfig().eta_ - 0.95));
  EXPECT_GE(ep, fabs(car_lon_config_.GetDiverConfig().a0_ - 400.0));
  EXPECT_GE(ep, fabs(car_lon_config_.GetDiverConfig().a1_ - 0.1));
  EXPECT_GE(ep, fabs(car_lon_config_.GetDiverConfig().a2_ - -0.0002));
  EXPECT_GE(ep, fabs(car_lon_config_.GetRollResistConfig().f_ - 0.012));
  EXPECT_GE(ep, fabs(car_lon_config_.GetAirResistConfig().Cd_ - 0.31));
  EXPECT_GE(ep, fabs(car_lon_config_.GetAirResistConfig().rho_ - 1.2258));
  EXPECT_GE(ep, fabs(car_lon_config_.GetAirResistConfig().A_) - 1.55);
  EXPECT_GE(ep, fabs(car_lon_config_.GetSlopeResistConfig().alpha_) - 0.0);
  EXPECT_GE(ep,
            fabs(car_lon_config_.GetAccelerationResistConfig().delte_ - 1.2));
  EXPECT_GE(ep, fabs(car_lon_config_.GetBrakeConfig().kb_ - 0.39));
}

TEST_F(FileToolTest, ReasPIDControllerConfig) {
  file_tool_.ReadPIDControllerConfig(pid_config_input_file_,
                                     &pid_controller_config_);
  double ep = 1e-6;
  EXPECT_GE(ep, fabs(pid_controller_config_.GetKp() - 1.5));
  EXPECT_GE(ep, fabs(pid_controller_config_.GetKi() - 0.5));
  EXPECT_GE(ep, fabs(pid_controller_config_.GetKd() - 0.));
  EXPECT_GE(ep, fabs(pid_controller_config_.GetIntSatHigh() - 0.3));
  EXPECT_GE(ep, fabs(pid_controller_config_.GetIntSatLow() + 0.3));
  EXPECT_GE(ep, fabs(pid_controller_config_.GetOutSatHigh() - 3.));
  EXPECT_GE(ep, fabs(pid_controller_config_.GetOutSatLow() + 3.));
}

}  // namespace tools
}  // namespace autopilot

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  // autopilot::tools::FileTool file_tool;
  // autopilot::configs::carconfig::CarLonConfig car_lon_config;

  // double m = atof("1.1");
  // car_lon_config.GetVehicleConfig().m_ = atof("1.0");

  // std::string config_file_path =
  //     "/home/ymm/mycode/autopilot/data/carlonconfiginput.txt";
  // std::ifstream config_input_file(config_file_path.c_str(), std::ios::in);
  // if (!config_input_file) {
  //   std::cout << "open error!" << std::endl;
  //   return 0;
  // }
  // int res = file_tool.ReadCarLonConfig(config_input_file, &car_lon_config);
  // if (res == -1) {
  //   std::cout << "error!" << std::endl;
  // } else {
  //   std::cout << "success!" << std::endl;
  // }
  return 0;
}