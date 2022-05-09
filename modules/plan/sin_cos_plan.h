/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 * @brief Generate sinusoidal ramp path
 *
 */

#pragma once

#include <vector>

/**
 * @namespace autopilot::modules::plan
 * @brief autopilot::modules::plan
 *
 */
namespace autopilot {
namespace modules {
namespace plan {

/**
 * @class SinRampPath
 * @brief Generate sinusoidal ramp path and cosine velocity
 *
 */
class SinRampPath {
 public:
  void Init(double w, double am, double ramp) {
    w_ = w;
    am_ = am;
    ramp_ = ramp;
  }

  /**
   * @brief Generate sinusoidal ramp path and cosine velocity
   *        total planning time = plan_times * dt
   *
   * @param plan_times total planning time steps
   * @param dt generating period / s
   */
  void Generate(int plan_time_steps, double dt);

  /**
   * @brief Get the Positions object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetPositions() const { return plan_positions_; }

  /**
   * @brief Get the Velocities object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetVelocities() const { return plan_velocities_; }

  /**
   * @brief Get the Times object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetTimes() const { return plan_times_; }

 private:
  double w_ = 0.0;
  double am_ = 0.0;
  double ramp_ = 0.0;
  std::vector<double> plan_positions_;
  std::vector<double> plan_velocities_;
  std::vector<double> plan_times_;
};

}  // namespace plan
}  // namespace modules
}  // namespace autopilot
