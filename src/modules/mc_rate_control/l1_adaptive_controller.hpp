#pragma once

#include <matrix/matrix/math.hpp>

// L1 adaptive augmentation (torque-level)
// Inputs:
//  - dt                 : timestep [s]
//  - tau_cmd_rpy        : commanded torque from baseline controller [Nm]
//  - omega_meas_rpy     : measured body angular rates [rad/s]
//  - lin_accel_body     : body linear acceleration used to form force (F = m*a) [m/s^2]
//
// Outputs:
//  - tau_tilde_rpy      : compensated torque command [Nm]
//  - dhat_tau_out       : disturbance observation output [Nm]
//  - tau_comp_raw_out   : raw compensation torque (F x p_hat) [Nm]
//  - tau_comp_lpf_out   : 1st-order LPF of compensation torque [Nm]
//
void l1_adaptive_controller(float dt,
                            const matrix::Vector3f &tau_cmd_rpy,
                            const matrix::Vector3f &omega_meas_rpy,
                            const matrix::Vector3f &lin_accel_body,
                            matrix::Vector3f &tau_tilde_rpy,
                            matrix::Vector3f &dhat_tau_out,
                            matrix::Vector3f &tau_comp_raw_out,
                            matrix::Vector3f &tau_comp_lpf_out,
                            matrix::Vector3f body_force_desired);
