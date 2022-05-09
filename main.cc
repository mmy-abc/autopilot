/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @brief The Main Funcation
 */

#include "./models/carmodel/car_lon_model.h"
#include "./modules/control/mpc_controller.h"
#include "./modules/control/pid_controller.h"
#include "./modules/plan/sin_cos_plan.h"
#include "constant.h"
#include "matplotlibcpp.h"

/**
 * @namespace autopilot
 * @brief autopilot
 */
namespace autopilot {}  // namespace autopilot

void TestPIDControl() {
  // car model
  std::string car_lon_config_file =
      "/home/ymm/mycode/autopilot/data/carlonconfiginput.txt";
  autopilot::models::carmodel::CarLonModel car_lon_model{car_lon_config_file};
  car_lon_model.Init();

  // pid controller
  std::string pid_controller_config_file =
      "/home/ymm/mycode/autopilot/data/pidcontrollerinput.txt";
  autopilot::modules::control::PIDController pid_controller_pos{
      pid_controller_config_file};
  pid_controller_pos.Init();
  // pid controller
  std::string pid_controller_config_file2 =

      "/home/ymm/mycode/autopilot/data/pidcontrollerinput2.txt";
  autopilot::modules::control::PIDController pid_controller_vel{
      pid_controller_config_file2};
  pid_controller_vel.Init();

  // plan positions and velocity
  autopilot::modules::plan::SinRampPath sin_ramp_path;
  double w = 2 * M_PI / 240;
  double am = 150.0;
  double ramp = 20.0;
  double dt = 0.01;
  int sim_steps = 60000;
  sin_ramp_path.Init(w, am, ramp);
  sin_ramp_path.Generate(sim_steps, dt);

  // run sim
  int i = 0;
  double error_pos = 0.0;
  double error_vel = 0.0;
  int loop = dt / car_lon_model.GetCarLonConfig().GetSimStep();
  std::vector<double> position_errors;
  std::vector<double> velocity_errors;
  std::vector<double> throttles;
  std::vector<double> brakes;
  while (i <= sim_steps) {
    error_pos = sin_ramp_path.GetPositions()[i] - car_lon_model.GetPosition();
    double vel_com = pid_controller_pos.Control(error_pos, dt);
    error_vel = sin_ramp_path.GetVelocities()[i] - car_lon_model.GetVelocity();
    double acc_com = pid_controller_vel.Control(error_vel + vel_com, dt);
    double time = i * dt;
    car_lon_model.times_.emplace_back(time);
    car_lon_model.positions_.emplace_back(car_lon_model.GetPosition());
    car_lon_model.velocitys_.emplace_back(car_lon_model.GetVelocity());
    position_errors.emplace_back(error_pos);
    velocity_errors.emplace_back(error_vel);
    for (int j = 0; j < loop; j++) {
      if (acc_com >= 0) {
        double throttle = acc_com * 0.1;
        if (throttle > 1.) {
          throttle = 1.;
        }
        car_lon_model.SetCarGear(autopilot::CarGear::forwoard);
        car_lon_model.SetThrottle(throttle);
      } else {
        double brake = -acc_com * 0.3;
        if (brake > 20.) {
          brake = 20.;
        }
        car_lon_model.SetCarGear(autopilot::CarGear::brake);
        car_lon_model.SetBrake(brake);
      }
      car_lon_model.Step();
    }
    if (acc_com >= 0) {
      double throttle = acc_com * 0.1;
      if (throttle > 1.) {
        throttle = 1.;
      }
      throttles.emplace_back(throttle);
      brakes.emplace_back(0.0);
    } else {
      double brake = -acc_com * 0.3;
      if (brake > 20.) {
        brake = 20.;
      }
      throttles.emplace_back(0.0);
      brakes.emplace_back(brake);
    }

    i++;
  }

  // plot
  namespace plt = matplotlibcpp;
  plt::figure();
  plt::plot(car_lon_model.times_, position_errors, "r");
  plt::xlabel("t/s");
  plt::ylabel("position-error/m");
  plt::title("position-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, velocity_errors, "r");
  plt::xlabel("t/s");
  plt::ylabel("velocity-error/ms-1");
  plt::title("velocity-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.positions_, "r");
  plt::plot(sin_ramp_path.GetTimes(), sin_ramp_path.GetPositions(), "g");
  plt::xlabel("t/s");
  plt::ylabel("position/m");
  plt::title("position-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.velocitys_, "r");
  plt::plot(sin_ramp_path.GetTimes(), sin_ramp_path.GetVelocities(), "g");
  plt::xlabel("t/s");
  plt::ylabel("velocity/ms-1");
  plt::title("velocity-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, throttles, "r");
  plt::xlabel("t/s");
  plt::ylabel("throttle");
  plt::title("throttle-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, brakes, "r");
  plt::xlabel("t/s");
  plt::ylabel("brake");
  plt::title("brake-time");
  plt::show();
}

void TestMPCControl() {
  // car model
  std::string car_lon_config_file =
      "/home/ymm/mycode/autopilot/data/carlonconfiginput.txt";
  autopilot::models::carmodel::CarLonModel car_lon_model{car_lon_config_file};
  car_lon_model.Init();

  // plan positions and velocity
  autopilot::modules::plan::SinRampPath sin_ramp_path;
  double w = 2 * M_PI / 240;
  double am = 150.0;
  double ramp = 20.0;
  double dt = 0.01;
  int sim_steps = 60000;
  sin_ramp_path.Init(w, am, ramp);
  sin_ramp_path.Generate(sim_steps, dt);

  // mpc controller
  Eigen::Vector3d x_ref;
  Eigen::VectorXd x0;
  autopilot::modules::control::MPCController mpc_controller(3, 1, 20);
  x_ref << sin_ramp_path.GetPositions()[0], sin_ramp_path.GetVelocities()[0],
      0.0;
  x0.resize(3);
  x0 << car_lon_model.GetPosition(), car_lon_model.GetVelocity(),
      car_lon_model.GetAcceleration();
  mpc_controller.Init(x0);
  mpc_controller.ControlInit(x_ref);

  // run sim
  int i = 0;
  int loop = dt / car_lon_model.GetCarLonConfig().GetSimStep();
  double error_pos = 0.0;
  double error_vel = 0.0;
  std::vector<double> position_errors;
  std::vector<double> velocity_errors;
  std::vector<double> throttles;
  std::vector<double> brakes;
  while (i <= sim_steps) {
    error_pos = sin_ramp_path.GetPositions()[i] - car_lon_model.GetPosition();
    error_vel = sin_ramp_path.GetVelocities()[i] - car_lon_model.GetVelocity();
    double time = i * dt;
    car_lon_model.times_.emplace_back(time);
    car_lon_model.positions_.emplace_back(car_lon_model.GetPosition());
    car_lon_model.velocitys_.emplace_back(car_lon_model.GetVelocity());
    position_errors.emplace_back(error_pos);
    velocity_errors.emplace_back(error_vel);

    Eigen::Vector3d x_ref;
    x_ref << sin_ramp_path.GetPositions()[i], sin_ramp_path.GetVelocities()[i],
        0.0;
    Eigen::VectorXd x0;
    x0.resize(3);
    x0 << car_lon_model.GetPosition(), car_lon_model.GetVelocity(),
        car_lon_model.GetAcceleration();
    mpc_controller.SetState(x0);
    mpc_controller.UpdateConstraintVectors();
    mpc_controller.UpdateGradient(x_ref);
    // mpc_controller.Debug();
    double acc_com = mpc_controller.Control();
    for (int j = 0; j < loop; j++) {
      if (acc_com >= 0) {
        double throttle = acc_com * 0.1;
        if (throttle > 1.) {
          throttle = 1.;
        }
        car_lon_model.SetCarGear(autopilot::CarGear::forwoard);
        car_lon_model.SetThrottle(throttle);
      } else {
        double brake = -acc_com * 0.3;
        if (brake > 20.) {
          brake = 20.;
        }
        car_lon_model.SetCarGear(autopilot::CarGear::brake);
        car_lon_model.SetBrake(brake);
      }
      car_lon_model.Step();
    }
    if (acc_com >= 0) {
      double throttle = acc_com * 0.1;
      if (throttle > 1.) {
        throttle = 1.;
      }
      throttles.emplace_back(throttle);
      brakes.emplace_back(0.0);
    } else {
      double brake = -acc_com * 0.3;
      if (brake > 20.) {
        brake = 20.;
      }
      throttles.emplace_back(0.0);
      brakes.emplace_back(brake);
    }

    i++;
  }

  // plot
  namespace plt = matplotlibcpp;
  plt::figure();
  plt::plot(car_lon_model.times_, position_errors, "r");
  plt::xlabel("t/s");
  plt::ylabel("position-error/m");
  plt::title("position-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, velocity_errors, "r");
  plt::xlabel("t/s");
  plt::ylabel("velocity-error/ms-1");
  plt::title("velocity-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.positions_, "r");
  plt::plot(sin_ramp_path.GetTimes(), sin_ramp_path.GetPositions(), "g");
  plt::xlabel("t/s");
  plt::ylabel("position/m");
  plt::title("position-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, car_lon_model.velocitys_, "r");
  plt::plot(sin_ramp_path.GetTimes(), sin_ramp_path.GetVelocities(), "g");
  plt::xlabel("t/s");
  plt::ylabel("velocity/ms-1");
  plt::title("velocity-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, throttles, "r");
  plt::xlabel("t/s");
  plt::ylabel("throttle");
  plt::title("throttle-time");
  plt::show();

  plt::figure();
  plt::plot(car_lon_model.times_, brakes, "r");
  plt::xlabel("t/s");
  plt::ylabel("brake");
  plt::title("brake-time");
  plt::show();
}

int main(int argc, char **argv) {
  // TestPIDControl();
  TestMPCControl();
  return autopilot::Constants::SUCCESS;
}
