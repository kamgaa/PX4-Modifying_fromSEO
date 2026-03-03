#pragma once

#include <matrix/matrix/math.hpp>

// L1 adaptive augmentation (torque-level, rate-domain predictor)
//
// Structure:
//   - Predictor:  J * ω_hat_dot = τ_applied + h_hat - ω_hat × (J ω_hat)
//   - Adaptation: h_hat_dot = -Γ (ω_hat - ω_meas)
//   - LPF:        h_f_dot   = ωc (h_hat - h_f)
//   - Control:    τ = τ_cmd - h_f
//
// Inputs:
//  - dt             : timestep [s]
//  - tau_cmd_rpy    : baseline torque command (body frame) [N*m]
//  - omega_meas_rpy : measured body angular rates (body frame) [rad/s]
//
// Outputs:
//  - tau_tilde_rpy     : compensated torque command [N*m]
//  - dhat_tau_out      : raw estimated matched disturbance torque (h_hat) [N*m]
//  - tau_comp_raw_out  : raw compensation signal (h_hat) [N*m] (for logging)
//  - tau_comp_lpf_out  : filtered compensation used in control (h_f) [N*m] (for logging)
//
void l1_adaptive_controller(float dt,
                            const matrix::Vector3f &tau_cmd_rpy,
                            const matrix::Vector3f &omega_meas_rpy,
                            matrix::Vector3f &tau_tilde_rpy,
                            matrix::Vector3f &dhat_tau_out,
                            matrix::Vector3f &tau_comp_raw_out,
                            matrix::Vector3f &tau_comp_lpf_out);

// Reset internal L1 states (call when L1 flag is OFF, on disarm, etc.)
void l1_adaptive_reset();