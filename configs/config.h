/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines Config Class
 */

#pragma once

#include "../constant.h"

/**
 * @namespace autopilot::configs
 * @brief autopilot::configs
 */
namespace autopilot {
namespace configs {
/**
 * @brief Parent Config Class
 *
 */
class Config {
 public:
  /**
   * @brief Construct a new Config object
   *
   */

  Config() = default;
  /**
   * @brief Destroy the Config object
   *
   */

  virtual ~Config() = default;

  virtual int ReadConfig() { return Constants::SUCCESS; }
};
}  // namespace configs
}  // namespace autopilot
