/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @brief Defines File Tool Class
 *
 */

#pragma once

#include <fstream>

#include "../configs/config.h"

/**
 * @namespace autopilot::tools
 * @brief autopilot::tools
 */
namespace autopilot {
namespace tools {
/**
 * @class FileTool
 * @brief File Tool Class
 *
 */
class FileTool {
 public:
  /**
   * @brief Construct a new File Tool object
   *
   */
  FileTool() = default;

  /**
   * @brief Destroy the File Tool object
   *
   */
  virtual ~FileTool() = default;

  /**
   * @brief Read car lon config file
   *
   * @param config_input_file
   * @param config
   * @return int
   */
  int ReadCarLonConfig(std::ifstream& config_input_file,
                       autopilot::configs::Config* config);

  int ReadPIDControllerConfig(std::ifstream& config_input_file,
                              autopilot::configs::Config* config);
};
}  // namespace tools
}  // namespace autopilot
