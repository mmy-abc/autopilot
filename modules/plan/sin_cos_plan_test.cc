/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/
#include "sin_cos_plan.h"

#include "../../matplotlibcpp.h"

int main() {
  autopilot::modules::plan::SinRampPath sin_ramp_path;
  double w = 2 * M_PI / 120;
  double am = 150.0;
  double ramp = 20.0;
  sin_ramp_path.Init(w, am, ramp);
  sin_ramp_path.Generate(60000, 0.01);

  namespace plt = matplotlibcpp;
  plt::figure();
  plt::plot(sin_ramp_path.GetTimes(), sin_ramp_path.GetPositions());
  plt::xlabel("t/s");
  plt::ylabel("position/m");
  plt::title("position-time");
  plt::show();

  plt::figure();
  plt::plot(sin_ramp_path.GetTimes(), sin_ramp_path.GetVelocities());
  plt::xlabel("t/s");
  plt::ylabel("velocity/ms-1");
  plt::title("velocity-time");
  plt::show();
}