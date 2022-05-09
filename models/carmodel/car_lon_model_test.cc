/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "car_lon_model.h"

#include <gtest/gtest.h>

#include <string>

#include "../../matplotlibcpp.h"

namespace autopilot {
namespace models {
namespace carmodel {
class CarLonModelTest : public ::testing::Test {
 public:
 protected:
  std::string config_file_path{
      "/home/ymm/mycode/autopilot/data/carlonconfiginput.txt"};
  CarLonModel car_lon_model{config_file_path};
};

TEST_F(CarLonModelTest, Init) {
  car_lon_model.Init();
  auto car_lon_config_ = car_lon_model.GetCarLonConfig();
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

  EXPECT_GE(ep, fabs(car_lon_model.GetTime() - 0.0));
  EXPECT_GE(ep, fabs(car_lon_model.GetThrottle() - 0.0));
  EXPECT_GE(ep, fabs(car_lon_model.GetBrake() - 0.0));
  EXPECT_GE(ep, fabs(car_lon_model.GetPosition() - 0.0));
  EXPECT_GE(ep, fabs(car_lon_model.GetVelocity() - 0.0));
}

TEST_F(CarLonModelTest, Step) {
  double ep1 = 1.e-9;
  double ep2 = 1.e-6;
  int time_step = 12000;
  int i = 0;
  car_lon_model.Init();
  car_lon_model.times_.emplace_back(car_lon_model.GetTime());
  car_lon_model.positions_.emplace_back(car_lon_model.GetPosition());
  car_lon_model.velocitys_.emplace_back(car_lon_model.GetVelocity());
  car_lon_model.accelerations_.emplace_back(car_lon_model.GetAcceleration());
  while (i < time_step) {
    if (i < time_step / 3) {
      car_lon_model.SetCarGear(CarGear::forwoard);  // set gear
      car_lon_model.SetThrottle(0.6);               // set throttle
    } else if (i < time_step * 2 / 3) {
      car_lon_model.SetCarGear(CarGear::standstill);  // set
    } else {
      car_lon_model.SetCarGear(CarGear::brake);  // set gear
      car_lon_model.SetBrake(5);                 // set brake
    }

    car_lon_model.Step();

    double time = car_lon_model.GetTime();
    if (fmod(time + ep1, 0.1) < ep2) {
      car_lon_model.positions_.emplace_back(car_lon_model.GetPosition());
      car_lon_model.velocitys_.emplace_back(car_lon_model.GetVelocity());
      car_lon_model.accelerations_.emplace_back(
          car_lon_model.GetAcceleration());
    }

    i++;
  }

  EXPECT_GE(ep1, fabs(car_lon_model.GetTime() - 60.0));
}
}  // namespace carmodel
}  // namespace models
}  // namespace autopilot

int main(int argc, char **argv) {
  // ::testing::InitGoogleTest(&argc, argv);
  // return RUN_ALL_TESTS();

  std::string config_file_path{
      "/home/ymm/mycode/autopilot/data/carlonconfiginput.txt"};
  autopilot::models::carmodel::CarLonModel car_lon_model{config_file_path};
  double ep1 = 1.e-9;
  double ep2 = 1.e-6;
  int time_step = 300000;
  int i = 0;
  car_lon_model.Init();
  car_lon_model.times_.emplace_back(car_lon_model.GetTime());
  car_lon_model.positions_.emplace_back(car_lon_model.GetPosition());
  car_lon_model.velocitys_.emplace_back(car_lon_model.GetVelocity());
  car_lon_model.accelerations_.emplace_back(car_lon_model.GetAcceleration());
  while (i < time_step) {
    if (i < time_step / 6) {
      car_lon_model.SetCarGear(autopilot::CarGear::forwoard);  // set gear
      car_lon_model.SetThrottle(0.6);                          // set throttle
    } else if (i < time_step * 2 / 3) {
      car_lon_model.SetCarGear(autopilot::CarGear::standstill);  // set
    } else {
      car_lon_model.SetCarGear(autopilot::CarGear::brake);  // set gear
      car_lon_model.SetBrake(15);                           // set brake
    }

    car_lon_model.Step();

    double time = car_lon_model.GetTime();
    if (fmod(time + ep1, 0.1) < ep2) {
      car_lon_model.times_.emplace_back(time);
      car_lon_model.positions_.emplace_back(car_lon_model.GetPosition());
      car_lon_model.velocitys_.emplace_back(car_lon_model.GetVelocity());
      car_lon_model.accelerations_.emplace_back(
          car_lon_model.GetAcceleration());
    }

    i++;
  }

  namespace plt = matplotlibcpp;
  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.positions_);
  plt::xlabel("t/s");
  plt::ylabel("position/m");
  plt::title("position-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.velocitys_);
  plt::xlabel("t/s");
  plt::ylabel("velocity/ms-1");
  plt::title("velocity-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.accelerations_);
  plt::xlabel("t/s");
  plt::ylabel("acceleration/ms-2");
  plt::title("acceleration-time");
  plt::show();
}