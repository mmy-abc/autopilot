/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "mpc_controller.h"

#include <cmath>
#include <iostream>

#include "../../constant.h"

namespace autopilot {
namespace modules {
namespace control {

int MPCController::Init(Eigen::VectorXd& state0) {
  // init state
  state_ = std::move(state0);
  // init cotrol
  control_.resize(control_dim_);
  control_ << 0.0;
  // init state constraint
  state_max_.resize(state_dim_);
  state_max_ << OsqpEigen::INFTY, OsqpEigen::INFTY, 10.0;
  state_min_.resize(state_dim_);
  state_min_ << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -10.0;
  // init control constraint
  control_max_.resize(control_dim_);
  control_max_ << 10.0;
  control_min_.resize(control_dim_);
  control_min_ << -10.0;
  // init control deta constraint
  control_deta_max_.resize(control_dim_);
  control_deta_max_ << 10;
  control_deta_min_.resize(control_dim_);
  control_deta_min_ << -10;
  // init A
  A_.resize(state_dim_, state_dim_);
  // A_ << 1.0, 0.1 * (1 - exp(-0.1)), 0.0, exp(-0.1);
  A_ << 1, 0.01, 0.1 * 0.1 * exp(-0.1) + 0.1 * 0.01 - 0.1 * 0.1, 0.0, 1.0,
      0.1 * (1 - exp(-0.1)), 0.0, 0.0, exp(-0.1);
  // init B
  B_.resize(state_dim_, control_dim_);
  // B_ << 0.01 - 0.1 + 0.1 * exp(-0.1), 1 - exp(-0.1);
  B_ << -0.1 * 0.1 * exp(-0.1) + 0.1 * 0.1 + 0.5 * 0.01 * 0.01 - 0.1 * 0.01,
      0.01 - 0.1 + 0.1 * exp(-0.1), 1 - exp(-0.1);
  // init Q
  Q_.resize(state_dim_);
  Q_.diagonal() << 100.0, 100.0, 0.0;
  // init R
  R_.resize(control_dim_);
  R_.diagonal() << 10.0;
  // init F
  F_.resize(state_dim_);
  F_.diagonal() << 100.0, 100.0, 0.0;

  return Constants::SUCCESS;
}

double MPCController::Control() {
  // QPSolution vector
  Eigen::VectorXd qp_solution;
  // solve the QP problem
  if (!solver_.solve()) return control_.norm();

  // get the controller input
  qp_solution = solver_.getSolution();
  control_ = qp_solution.block(state_dim_ * (mpc_n_ + 1), 0, control_dim_, 1);

  return control_.norm();
}

int MPCController::ControlInit(const Eigen::MatrixXd& x_ref) {
  // settings
  // solver.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  int n = state_dim_ * (mpc_n_ + 1);
  int m = control_dim_ * mpc_n_;
  solver_.data()->setNumberOfVariables(n + m);
  solver_.data()->setNumberOfConstraints(2 * n + m);
  if (CalcQPHessian()) return Constants::FAIL;
  if (CalcQPGradient(x_ref)) return Constants::FAIL;
  if (CalcQPConstraintMatrices()) return Constants::FAIL;
  if (CalcQPConstraintVectors()) return Constants::FAIL;

  if (!solver_.data()->setHessianMatrix(hessian_matrix_))
    return Constants::FAIL;
  if (!solver_.data()->setGradient(gradient_)) return Constants::FAIL;
  if (!solver_.data()->setLinearConstraintsMatrix(constraint_matrix_))
    return Constants::FAIL;
  if (!solver_.data()->setLowerBound(lower_bound_)) return Constants::FAIL;
  if (!solver_.data()->setUpperBound(upper_bound_)) return Constants::FAIL;

  // instantiate the solver
  if (!solver_.initSolver()) return Constants::FAIL;

  return Constants::SUCCESS;
}

int MPCController::CalcQPHessian() {
  int n = state_dim_ * mpc_n_;
  int m = control_dim_ * mpc_n_;
  hessian_matrix_.resize(n + state_dim_ + m, n + state_dim_ + m);

  for (int i = 0; i < n + m + state_dim_; i++) {
    double value = 0.0;
    if (i < n) {
      int pos_Q = i % state_dim_;
      value = Q_.diagonal()[pos_Q];
    } else if (i < n + state_dim_) {
      int pos_F = (i - n) % state_dim_;
      value = F_.diagonal()[pos_F];
    } else {
      int pos_R = (i - n - state_dim_) % control_dim_;
      value = R_.diagonal()[pos_R];
    }
    if (value != 0) {
      hessian_matrix_.insert(i, i) = value;
    }
  }

  return Constants::SUCCESS;
}

int MPCController::CalcQPGradient(const Eigen::MatrixXd& x_ref) {
  Eigen::VectorXd Qx_ref;
  Qx_ref.resize(state_dim_);
  Qx_ref = Q_ * (-x_ref);

  int n = state_dim_ * (mpc_n_ + 1);
  int m = control_dim_ * mpc_n_;
  gradient_ = Eigen::VectorXd::Zero(n + m, 1);

  for (int i = 0; i < n; i++) {
    int pos_Q = i % state_dim_;
    double value = Qx_ref(pos_Q, 0);
    gradient_(i, 0) = value;
  }

  return Constants::SUCCESS;
}

int MPCController::CalcQPConstraintMatrices() {
  int n = state_dim_ * (mpc_n_ + 1);
  int m = control_dim_ * mpc_n_;
  constraint_matrix_.resize(2 * n + m, n + m);

  // equality constrant
  for (int i = 0; i < n; i++) {
    constraint_matrix_.insert(i, i) = -1;
  }
  for (int i = 0; i < mpc_n_; i++) {
    for (int j = 0; j < state_dim_; j++) {
      for (int k = 0; k < state_dim_; k++) {
        float value = A_(j, k);
        if (value != 0) {
          constraint_matrix_.insert(state_dim_ * (i + 1) + j,
                                    state_dim_ * i + k) = value;
        }
      }
    }
  }
  for (int i = 0; i < mpc_n_; i++) {
    for (int j = 0; j < state_dim_; j++) {
      for (int k = 0; k < control_dim_; k++) {
        float value = B_(j, k);
        if (value != 0) {
          constraint_matrix_.insert(state_dim_ * (i + 1) + j,
                                    n + control_dim_ * i + k) = value;
        }
      }
    }
  }

  // inequality constrant
  for (int i = 0; i < n + m; i++) {
    constraint_matrix_.insert(n + i, i) = 1;
  }

  return Constants::SUCCESS;
}

int MPCController::CalcQPConstraintVectors() {
  int n = state_dim_ * (mpc_n_ + 1);
  int m = control_dim_ * mpc_n_;

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lower_equality = Eigen::MatrixXd::Zero(n, 1);
  Eigen::VectorXd upper_equality;
  lower_equality.block(0, 0, state_dim_, 1) = -state_;
  upper_equality = lower_equality;
  lower_equality = lower_equality;

  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lower_inequality = Eigen::MatrixXd::Zero(n + m, 1);
  Eigen::VectorXd upper_inequality = Eigen::MatrixXd::Zero(n + m, 1);
  for (int i = 0; i < mpc_n_ + 1; i++) {
    lower_inequality.block(state_dim_ * i, 0, state_dim_, 1) = state_min_;
    upper_inequality.block(state_dim_ * i, 0, state_dim_, 1) = state_max_;
  }
  for (int i = 0; i < mpc_n_; i++) {
    lower_inequality.block(n + control_dim_ * i, 0, control_dim_, 1) =
        control_min_;
    upper_inequality.block(n + control_dim_ * i, 0, control_dim_, 1) =
        control_max_;
  }

  // merge inequality and equality vectors
  lower_bound_.resize(2 * n + m);
  upper_bound_.resize(2 * n + m);
  lower_bound_ << lower_equality, lower_inequality;
  upper_bound_ << upper_equality, upper_inequality;

  return Constants::SUCCESS;
}

int MPCController::UpdateConstraintVectors() {
  lower_bound_.block(0, 0, state_dim_, 1) = -state_;
  upper_bound_.block(0, 0, state_dim_, 1) = -state_;

  if (!solver_.updateBounds(lower_bound_, upper_bound_)) {
    return Constants::FAIL;
  }

  return Constants::SUCCESS;
}

int MPCController::UpdateGradient(const Eigen::MatrixXd& x_ref) {
  CalcQPGradient(x_ref);
  if (!solver_.updateGradient(gradient_)) {
    return Constants::FAIL;
  }

  return Constants::SUCCESS;
}

void MPCController::Debug() {
  std::cout << "A=\n" << A_ << std::endl;
  std::cout << "B=\n" << B_ << std::endl;
  Eigen::MatrixXd Q = Q_;
  Eigen::MatrixXd R = R_;
  Eigen::MatrixXd F = F_;
  Eigen::MatrixXd hessian_matrix = hessian_matrix_;
  Eigen::MatrixXd constraint_matrix = constraint_matrix_;
  std::cout << "Q=\n" << Q << std::endl;
  std::cout << "R=\n" << R << std::endl;
  std::cout << "F=\n" << F << std::endl;
  std::cout << "hessian_matrix=\n" << hessian_matrix << std::endl;
  std::cout << "constraint_matrix=\n" << constraint_matrix << std::endl;

  std::cout << "gradient=\n" << gradient_ << std::endl;
  std::cout << "state=\n" << state_ << std::endl;
  std::cout << "state_max=\n" << state_max_ << std::endl;
  std::cout << "state_min=\n" << state_min_ << std::endl;
  std::cout << "control=\n" << control_ << std::endl;
  std::cout << "control_max=\n" << control_max_ << std::endl;
  std::cout << "control_min=\n" << control_min_ << std::endl;
  std::cout << "control_deta_max=\n" << control_deta_max_ << std::endl;
  std::cout << "control_deta_min=\n" << control_deta_min_ << std::endl;
  std::cout << "lower_bound=\n" << lower_bound_ << std::endl;
  std::cout << "upper_bound=\n" << upper_bound_ << std::endl;
}
}  // namespace control
}  // namespace modules
}  // namespace autopilot