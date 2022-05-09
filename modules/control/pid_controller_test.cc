/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "pid_controller.h"

#include <gtest/gtest.h>

#include "../../matplotlibcpp.h"

namespace autopilot {
namespace modules {
namespace control {
class PIDControllerTest : public ::testing::Test {
 protected:
  std::string config_file_path_{
      "/home/ymm/mycode/autopilot/data/pidcontrollerinput.txt"};
  PIDController pid_controller{config_file_path_};
};

TEST_F(PIDControllerTest, Control) {
  pid_controller.Init();
  pid_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(0.1, dt), 0.1505, 1e-6);
  pid_controller.Reset();
  double control_value = pid_controller.Control(-0.1, dt);
  EXPECT_NEAR(control_value, -0.1505, 1e-6);
  dt = 0.0;
  EXPECT_EQ(pid_controller.Control(100, dt), control_value);
  EXPECT_FALSE(pid_controller.GetIntegralHold());
}
}  // namespace control
}  // namespace modules
}  // namespace autopilot

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  //   std::string config_file_path_{
  //       "/home/ymm/mycode/autopilot/data/pidcontrollerinput.txt"};
  //   autopilot::modules::control::PIDController
  //   pid_controller{config_file_path_}; pid_controller.Init();
  //   pid_controller.Reset();
  //   double dt = 0.01;
  //   double out1 = pid_controller.Control(0.0, dt);
  //   pid_controller.Reset();
  //   double out2 = pid_controller.Control(0.1, dt);
  //   pid_controller.Reset();
  //   double out3 = pid_controller.Control(-0.1, dt);

  return 0;
}