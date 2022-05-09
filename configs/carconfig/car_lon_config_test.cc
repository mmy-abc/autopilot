/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "car_lon_config.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace autopilot {
namespace configs {
namespace carconfig {
class CarLonConfigTest : public ::testing::Test {
 protected:
  std::string path{"/home/ymm/mycode/autopilot/data/carlonconfiginput.txt"};
  CarLonConfig car_lon_config_{path};
};

TEST_F(CarLonConfigTest, ReadConfig) {
  car_lon_config_.ReadConfig();
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

}  // namespace carconfig
}  // namespace configs
}  // namespace autopilot

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}