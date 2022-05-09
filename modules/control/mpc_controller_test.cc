/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "mpc_controller.h"

using namespace autopilot::modules::control;

int main() {
  Eigen::Vector2d x_ref;
  Eigen::VectorXd x0;
  x_ref << 0.0, 10.0;
  x0.resize(2);
  x0 << 0.0, 0.0;
  MPCController mpc_controller(2, 1, 2);
  mpc_controller.Init(x0);
  mpc_controller.ControlInit(x_ref);
  mpc_controller.Debug();
}