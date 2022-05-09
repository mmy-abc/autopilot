/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "car_lon_model.h"

#include <cmath>

namespace autopilot {
namespace models {
namespace carmodel {

int CarLonModel::Init() {
  if (car_lon_config_.ReadConfig() == Constants::FAIL) {
    return Constants::FAIL;
  }

  position_ = 0.0;
  velocity_ = 0.0;
  acceleration_ = 0.0;
  prev_velocity_ = 0.0;
  time_ = 0.0;
  throttle_ = 0.0;
  brake_ = 0.0;
  gear_ = CarGear::standstill;

  SetX();

  return Constants::SUCCESS;
}

int CarLonModel::SetX() {
  x_all_[0] = position_;
  x_all_[1] = velocity_;
  x_all_[2] = time_;
  return Constants::SUCCESS;
}

int CarLonModel::SetDX() {
  d_x_all_[0] = velocity_;
  // calculate resistance
  double Ff = car_lon_config_.GetRollResistConfig().f_ *
              car_lon_config_.GetVehicleConfig().m_ *
              Constants::G;  // rolling resistance
  double Fw = 0.5 * car_lon_config_.GetAirResistConfig().Cd_ *
              car_lon_config_.GetAirResistConfig().A_ *
              car_lon_config_.GetAirResistConfig().rho_ * (velocity_ + 5.0) *
              (velocity_ +
               5.0);  // air resistance, suppose wind velocity is constant: 5m/s
  double Fi =
      sin(car_lon_config_.GetSlopeResistConfig().alpha_ * Constants::A2R) *
      car_lon_config_.GetVehicleConfig().m_ * Constants::G;  // slope resistance
  double Fj = car_lon_config_.GetAccelerationResistConfig().delte_ *
              car_lon_config_.GetVehicleConfig().m_ *
              (velocity_ - prev_velocity_) *
              car_lon_config_.GetSimStep();  // acceleration resistance
  double a = 0.0;                            // total acceleration
  if (gear_ == CarGear::forwoard) {
    // calculate driving force
    double wheel_omega =
        velocity_ /
        car_lon_config_.GetVehicleConfig().r_;  // wheel angular velocity
    double engine_omega =
        wheel_omega / car_lon_config_.GetDiverConfig().ig_ /
        car_lon_config_.GetDiverConfig().i0_ /
        car_lon_config_.GetDiverConfig().eta_;  // engine angular velocity
    double Ttq = throttle_ * car_lon_config_.GetDiverConfig().a0_ +
                 car_lon_config_.GetDiverConfig().a1_ * engine_omega +
                 car_lon_config_.GetDiverConfig().a2_ * engine_omega *
                     engine_omega;  // engine torque
    double Ft = Ttq * car_lon_config_.GetDiverConfig().ig_ *
                car_lon_config_.GetDiverConfig().i0_ *
                car_lon_config_.GetDiverConfig().eta_ /
                car_lon_config_.GetVehicleConfig().r_;  // driving force
    if (velocity_ >= 0) {
      a = (Ft - Ff - Fw - Fi - Fj) / car_lon_config_.GetVehicleConfig().m_;
    } else {
      a = (-Ft + Ff + Fw + Fi + Fj) / car_lon_config_.GetVehicleConfig().m_;
    }
    d_x_all_[1] = a;
  } else if (gear_ == autopilot::CarGear::brake) {
    // calculate braking force
    double Fb = brake_ * car_lon_config_.GetBrakeConfig().kb_;
    if (velocity_ >= 0) {
      a = (-Fb - Ff - Fw - Fi - Fj) / car_lon_config_.GetVehicleConfig().m_;
    } else {
      a = (Fb + Ff + Fw + Fi + Fj) / car_lon_config_.GetVehicleConfig().m_;
    }
  } else if (gear_ == autopilot::CarGear::standstill) {
    if (velocity_ >= 0) {
      a = (0.0 - Ff - Fw - Fi - Fj) / car_lon_config_.GetVehicleConfig().m_;
    } else {
      a = (Ff + Fw + Fi + Fj) / car_lon_config_.GetVehicleConfig().m_;
    }
  }
  d_x_all_[1] = a;
  d_x_all_[2] = 1;

  return Constants::SUCCESS;
}

int CarLonModel::ReSetStates() {
  position_ = 0.0;
  velocity_ = 0.0;
  acceleration_ = 0.0;
  prev_velocity_ = 0.0;
  time_ = 0.0;
  throttle_ = 0.0;
  brake_ = 0.0;
  gear_ = CarGear::standstill;

  SetX();

  return Constants::SUCCESS;
}

int CarLonModel::Update() {
  prev_velocity_ = velocity_;
  position_ = x_all_[0];
  velocity_ = x_all_[1];
  time_ = x_all_[2];

  return Constants::SUCCESS;
}

int CarLonModel::Step() {
  double a[5];
  a[0] = a[1] = a[4] = car_lon_config_.GetSimStep() * 0.5;
  a[2] = a[3] = car_lon_config_.GetSimStep();
  double x1[Constants::CAR_LON_X_DIM];
  double x2[Constants::CAR_LON_X_DIM];
  double xw[Constants::CAR_LON_X_DIM];
  for (int i = 0; i < Constants::CAR_LON_X_DIM; i++) {
    x1[i] = x2[i] = xw[i] = x_all_[i];
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < Constants::CAR_LON_X_DIM; j++) {
      x_all_[j] = x2[j];
    }
    Update();
    SetDX();
    for (int j = 0; j < Constants::CAR_LON_X_DIM; j++) {
      x2[j] = a[i] * d_x_all_[j] + x1[j];
      xw[j] = a[i + 1] * d_x_all_[j] / 3.0 + xw[j];
    }
  }

  for (int i = 0; i < Constants::CAR_LON_X_DIM; i++) {
    x_all_[i] = xw[i];
  }
  Update();
  SetDX();
  acceleration_ = d_x_all_[1];

  return Constants::SUCCESS;
}

// {
//     a[0] = a[1] = a[4] = step / 2.0
//         a[2] = a[3] = step
//         x1 = self.xall.copy()
//         x2 = self.xall.copy()
//         xw = self.xall.copy()
//         for i in range(4):
//             for j in range(self.Dim):
//                 self.xall[j] = x2[j]
//             self.update(action, tgod)
//             self.dx()
//             d = self.dxall
//             for j in range(self.Dim):
//                 x2[j] = a[i] * d[j] + x1[j]
//                 xw[j] = a[i + 1] * d[j] / 3.0 + xw[j]
//             tgod = tgod - step * 0.25
//         for i in range(self.Dim):
//             self.xall[i] = xw[i]

//         self.update(action, tgod)
// }

}  // namespace carmodel
}  // namespace models
}  // namespace autopilot