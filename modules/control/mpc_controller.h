/******************************************************************************
 * Copyright 2022 YMM. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

/**
 * @file
 *
 * @brief Defines the MPCController class
 */

#pragma once

// osqp-eigen
#include <Eigen/Dense>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "controller.h"

/**
 * @namespace autopilot::modules::control
 * @brief autopilot::modules::control
 *
 */
namespace autopilot {
namespace modules {
namespace control {
class MPCController : public Controller {
 public:
  /**
   * @brief Construct a new MPCController object
   *
   */
  MPCController() = default;

  /**
   * @brief Construct a new MPCController object
   *
   * @param state_dim
   * @param control_dim
   * @param mpc_n
   */
  MPCController(int state_dim, int control_dim, int mpc_n) {
    state_dim_ = state_dim;
    control_dim_ = control_dim;
    mpc_n_ = mpc_n;
  }

  /**
   * @brief Init state_, control_, A, B, Q, R x_n_step_prediction
   * the system equation:
   * x(k+1)=Ax(k)+Bu(k)
   * A=[1,tao*(1-exp(-k*Ts/tao))/k;0,exp(-k*Ts/tao)]
   * B=[Ts+tao*exp(-k*Ts/tao)/tao-tao/k;1-exp(-k*Ts/tao)]
   * suppose sample period Ts=0.01, time constant tao=0.1, amplification factor
   * k=1
   *
   * @param state0 init state
   * @return int
   */
  int Init(Eigen::VectorXd& state0);

  /**
   * @brief Calculate control using MPC controller
   *
   * @return double
   */
  double Control();

  /**
   * @brief control init
   *
   * @param x_ref
   * @return int
   */
  int ControlInit(const Eigen::MatrixXd& x_ref);

  /**
   * @brief cast MPC to QP Hessian
   *
   * @return int
   */
  int CalcQPHessian();

  /**
   * @brief cast MPC to QP Gradient
   *
   * @param x_ref
   * @return int
   */
  int CalcQPGradient(const Eigen::MatrixXd& x_ref);

  /**
   * @brief cast MPC to QP Constraint Matrices
   *
   * @return int
   */
  int CalcQPConstraintMatrices();

  /**
   * @brief cast MPC to QP Constraint Vectors
   *
   * @return int
   */
  int CalcQPConstraintVectors();

  /**
   * @brief update constraint vectors
   *
   * @return int
   */
  int UpdateConstraintVectors();

  /**
   * @brief update gradient
   *
   * @param x_ref
   * @return int
   */
  int UpdateGradient(const Eigen::MatrixXd& x_ref);

  /**
   * @brief Set the State Dim object
   *
   * @param state_dim
   */
  void SetStateDim(int state_dim) { state_dim_ = state_dim; }

  /**
   * @brief Set the Control Dim object
   *
   * @param control_dim
   */
  void SetControlDim(int control_dim) { control_dim_ = control_dim; }

  /**
   * @brief
   *
   * @param mpc_n
   */
  void SetMPCN(int mpc_n) { mpc_n_ = mpc_n; }

  /**
   * @brief Set the State object
   *
   * @param state
   */
  void SetState(Eigen::VectorXd& state) { state_ = std::move(state); }

  /**
   * @brief Set the Control object
   *
   * @param control
   */
  void SetControl(Eigen::VectorXd& control) { control_ = std::move(control); }

  /**
   * @brief Set the State Max object
   *
   * @param state_max
   */
  void SetStateMax(Eigen::VectorXd& state_max) {
    state_max_ = std::move(state_max);
  }

  /**
   * @brief Set the State Min object
   *
   * @param state_min
   */
  void SetStateMin(Eigen::VectorXd& state_min) {
    state_min_ = std::move(state_min);
  }

  /**
   * @brief Set the Control Max object
   *
   * @param control_max
   */
  void SetControlMax(Eigen::VectorXd& control_max) {
    control_max_ = std::move(control_max);
  }

  /**
   * @brief Set the Control Min object
   *
   * @param control_min
   */
  void SetControlMin(Eigen::VectorXd& control_min) {
    control_min_ = std::move(control_min);
  }

  /**
   * @brief Set the Control Deta Max object
   *
   * @param control_deta_max
   */
  void SetControlDetaMax(Eigen::VectorXd& control_deta_max) {
    control_deta_max_ = std::move(control_deta_max);
  }

  /**
   * @brief Set the Control Deta Min object
   *
   * @param control_deta_min
   */
  void SetControlDetaMin(Eigen::VectorXd& control_deta_min) {
    control_deta_min_ = std::move(control_deta_min);
  }

  /**
   * @brief
   *
   * @param A
   */
  void SetA(Eigen::MatrixXd& A) { A_ = std::move(A); }

  /**
   * @brief
   *
   * @param B
   */
  void SetB(Eigen::MatrixXd& B) { B_ = std::move(B); }

  /**
   * @brief
   *
   * @param Q
   */
  void SetQ(Eigen::DiagonalMatrix<double, -1>& Q) { Q_ = std::move(Q); }

  /**
   * @brief
   *
   * @param R
   */
  void SetR(Eigen::DiagonalMatrix<double, -1>& R) { R_ = std::move(R); }

  /**
   * @brief
   *
   * @param F
   */
  void SetF(Eigen::DiagonalMatrix<double, -1>& F) { F_ = std::move(F); }

  void Debug();

  /**
   * @brief Destroy the MPCController object
   *
   */
  virtual ~MPCController() = default;

 private:
  int state_dim_;
  int control_dim_;
  int mpc_n_;                    // mpc controller prediction window size
  Eigen::VectorXd state_;        // system state: 2D velocity and acceleration
  Eigen::VectorXd control_;      // systeom control input 1D desire acceleration
  Eigen::VectorXd state_max_;    // state max constraint
  Eigen::VectorXd state_min_;    // state min constraint
  Eigen::VectorXd control_max_;  // control max constraint
  Eigen::VectorXd control_min_;  // control min constraint
  Eigen::VectorXd control_deta_max_;     // control deta max constraint
  Eigen::VectorXd control_deta_min_;     // control deta min constraint
  Eigen::MatrixXd A_;                    // system matrix
  Eigen::MatrixXd B_;                    // control matrix
  Eigen::DiagonalMatrix<double, -1> Q_;  // state weight matrix
  Eigen::DiagonalMatrix<double, -1> R_;  // control weight matix
  Eigen::DiagonalMatrix<double, -1>
      F_;  // final state weight matrix int prediction
  Eigen::SparseMatrix<double> hessian_matrix_;     // QP hessian matrix
  Eigen::VectorXd gradient_;                        // QP gradient vector
  Eigen::SparseMatrix<double> constraint_matrix_;  // QP linear matrix
  Eigen::VectorXd lower_bound_;                    // QP lower bound
  Eigen::VectorXd upper_bound_;                    // QP upper bound
  OsqpEigen::Solver solver_;
};
}  // namespace control
}  // namespace modules
}  // namespace autopilot