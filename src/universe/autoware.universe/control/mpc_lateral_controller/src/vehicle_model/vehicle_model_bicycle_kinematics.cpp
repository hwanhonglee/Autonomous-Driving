// Copyright 2018-2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include <cmath>

namespace autoware::motion::control::mpc_lateral_controller
{
KinematicsBicycleModel::KinematicsBicycleModel(
  const double wheelbase, const double steer_lim, const double steer_tau)
: VehicleModelInterface(/* dim_x */ 3, /* dim_u */ 1, /* dim_y */ 2, wheelbase)
{
  m_steer_lim = steer_lim;
  m_steer_tau = steer_tau;
}

void KinematicsBicycleModel::calculateDiscreteMatrix(
  Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
  const double dt)
{
  // auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

  /* Linearize delta around delta_r (reference delta) */
  // double delta_r = atan(m_wheelbase * m_curvature);
  // if (std::abs(delta_r) >= m_steer_lim) {
  //   delta_r = m_steer_lim * static_cast<double>(sign(delta_r));
  // }

  //HH_231015_1 // if using Koopman, annotation here _2  1: org, 2: optimal data
  // double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));
  // double velocity = m_velocity;
  // if (std::abs(m_velocity) < 1e-04) {
  //   velocity = 1e-04 * (m_velocity >= 0 ? 1 : -1);
  // }

  // HH_231015_1 autoware mpc 
  // a_d << 0.0, velocity, 0.0, 0.0, 0.0, velocity / m_wheelbase * cos_delta_r_squared_inv, 0.0, 0.0,
  //   -1.0 / m_steer_tau;

  // b_d << 0.0, 0.0, 1.0 / m_steer_tau;

  // c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  // w_d << 0.0,
  //   -velocity * m_curvature +
  //     velocity / m_wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
  //   0.0;
    
  //EDMDc HJ_231214
  // a_d << 1.000, -0.2155, -0.1910,
  //         0.0, 0.9961, -0.0052,
  //         0.0, -0.0087, 1.0102;

  // b_d << 0.4995,
  //       0.0202,
  //       -0.0154;

  // c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  // w_d << 0.0,
  //       0.0,
  //       0.0; 
  
  // EDMDc last
  // a_d << 1.000, 0.0004, 0.0590,
  //         0.0, 0.999, -0.021,
  //         0.0, 0.0, 0.989;

  // b_d << -0.033,
  //         0.0119,
  //         0.0108;

  // c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  // w_d << 0.0,
  //       0.0,
  //       0.0; 
        
  // DMDc HH_231121
  // a_d << 1.0, 0.011, -0.015,
  //      -0.00009, 0.757, 0.090,
  //      0.00003, 0.087, 0.965;
  // b_d << -0.028,
  //        0.262,
  //        -0.093;
  // c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  // w_d << 0.0,
  //        0.0,
  //        0.0; 

  //HH_231015_2
  // Koopman Neural Network | traning data_loss_0.03 (w/ yaw error) 
  a_d << -0.19835101, -0.3375339, -0.12927282, 
        0.24552305, 0.11873358, -0.13700576,
        0.1340872, 0.24476781, 0.10393731;

  b_d << -0.49352682,
          0.36990735,
          -0.00790359;

  c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  w_d << 0.0, 0.0, 0.0;


  // HJ_240105 00:05 ~ 240109
  // a_d <<  1.000, 0.0004, 0.0590,
  //         0.0, 0.999, -0.021,
  //         0.0, 0.0, 0.989;

  // b_d << -0.033,
  //       0.0119,
  //       0.0108;

  // c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  // w_d << 0.0,
  //        0.0,
  //        0.0;


  // HJ_240114 --> error states
  // a_d <<  0.9985, 0.0002 -0.0050,
  //         0.0, 0.9946, 0.0077,
  //         0.00001, -0.0004, 0.9893;

  // b_d << 0.0062,
  //       -0.0080,
  //       0.0105;

  // c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  // w_d << 0.0,
  //        0.0,
  //        0.0;

  // std::ignore = dt;

  // bilinear discretization for ZOH system
  // no discretization is needed for Cd
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  const Eigen::MatrixXd i_dt2a_inv = (I - dt * 0.5 * a_d).inverse();
  a_d = i_dt2a_inv * (I + dt * 0.5 * a_d);
  b_d = i_dt2a_inv * b_d * dt;
  w_d = i_dt2a_inv * w_d * dt;
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd & u_ref)
{
  u_ref(0, 0) = std::atan(m_wheelbase * m_curvature);
}
}  // namespace autoware::motion::control::mpc_lateral_controller