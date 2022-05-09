/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "pid_controller_config.h"

namespace autopilot {
namespace configs {
namespace controllerconfig {
int PIDControllerConfig::ReadConfig() {
  return file_tool_.ReadPIDControllerConfig(config_input_file_, this);
}
}  // namespace controllerconfig
}  // namespace configs
}  // namespace autopilot