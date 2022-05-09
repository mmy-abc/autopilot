/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/
#include "sin_cos_plan.h"

#include <cmath>

namespace autopilot {
namespace modules {
namespace plan {
void SinRampPath::Generate(int plan_time_steps, double dt) {
  double time = 0.0, velocity = 0.0, position = 0.0;
  for (int i = 0; i <= plan_time_steps; i++) {
    time = i * dt;
    velocity = am_ * w_ * cos(w_ * time) + ramp_;
    position = am_ * sin(w_ * time) + ramp_ * time;
    plan_times_.emplace_back(time);
    plan_velocities_.emplace_back(velocity);
    plan_positions_.emplace_back(position);
  }
}
}  // namespace plan
}  // namespace modules
}  // namespace autopilot