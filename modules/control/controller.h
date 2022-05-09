/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines Controller Class
 */

#pragma once

/**
 * @brief
 *
 */
/**
 * @namespace autopilot::modules::control
 * @brief autopilot::modules::control
 *
 */
namespace autopilot {
namespace modules {
namespace control {

/**
 * @class Controller
 * @brief The parent controller class
 *
 */
class Controller {
 public:
  /**
   * @brief Construct a new Controller object
   *
   */
  Controller() = default;

  /**
   * @brief Destroy the Controller object
   *
   */
  virtual ~Controller() = default;
};
}  // namespace control
}  // namespace modules
}  // namespace autopilot