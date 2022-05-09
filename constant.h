/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines common constants
 */

#pragma once

#include <cmath>

/**
 * @namespace autopilot
 * @brief autopilot
 */
namespace autopilot {
/**
 * @brief Some common constans
 *
 */
class Constants {
 public:
  static constexpr double A2R = M_PI / 180.0;
  static constexpr double G = 9.81;  // gravitational acceleration
  static const int SUCCESS = 0;      // successful return flag
  static const int FAIL = 1;         // failful return flag
  static const int CAR_LON_X_DIM =
      3;  // dim of the state of ongitudinal dynamics model of automobile
};

/**
 * @brief Car gear enum class
 *
 */
enum CarGear { standstill, forwoard, brake };
}  // namespace autopilot
