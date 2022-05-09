/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "car_lon_config.h"

#include "../../constant.h"

namespace autopilot {
namespace configs {
namespace carconfig {

int CarLonConfig::ReadConfig() {
  return file_tool_.ReadCarLonConfig(config_input_file_, this);
}

}  // namespace carconfig
}  // namespace configs
}  // namespace autopilot
