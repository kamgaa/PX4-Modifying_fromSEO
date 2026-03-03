#include "l1_adaptive_controller.hpp"

#include <mathlib/math/Functions.hpp>
#include <cmath>

using matrix::Vector3f;

namespace {
static bool initialized = false;

static Vector3f omega_hat(0.f, 0.f, 0.f); // predictor state: rate
static Vector3f h_hat(0.f, 0.f, 0.f);     // adaptive estimate (raw)
static Vector3f h_f(0.f, 0.f, 0.f);       // LPF output (= used in control)
} // namespace

// ===================================================================================== //

void l1_adaptive_reset()
{
	initialized = false;
	omega_hat = Vector3f(0.f, 0.f, 0.f);
	h_hat     = Vector3f(0.f, 0.f, 0.f);
	h_f       = Vector3f(0.f, 0.f, 0.f);
}

// ===================================================================================== //

void l1_adaptive_controller(float dt,
                            const Vector3f &tau_cmd_rpy,
                            const Vector3f &omega_meas_rpy,
                            Vector3f &tau_tilde_rpy,
                            Vector3f &dhat_tau_out,
                            Vector3f &tau_comp_raw_out,
                            Vector3f &tau_comp_lpf_out)
{
	// tau_cmd_rpy      : body-frame baseline controller torque [N*m]
	// omega_meas_rpy   : body-frame measured angular rate [rad/s]
	// tau_tilde_rpy    : final torque command = tau_cmd - h_f
	// dhat_tau_out     : h_hat (raw estimate)
	// tau_comp_raw_out : h_hat (raw compensation signal for logging)
	// tau_comp_lpf_out : h_f   (filtered compensation used in control)

	// -----------------------------
	// 0) sanity
	// -----------------------------
	if (!PX4_ISFINITE(dt) || dt <= 0.0f || dt > 0.05f ||
	    !PX4_ISFINITE(omega_meas_rpy(0)) || !PX4_ISFINITE(omega_meas_rpy(1)) || !PX4_ISFINITE(omega_meas_rpy(2)) ||
	    !PX4_ISFINITE(tau_cmd_rpy(0))    || !PX4_ISFINITE(tau_cmd_rpy(1))    || !PX4_ISFINITE(tau_cmd_rpy(2))) {

		tau_tilde_rpy     = tau_cmd_rpy;
		dhat_tau_out      = Vector3f(0.f, 0.f, 0.f);
		tau_comp_raw_out  = Vector3f(0.f, 0.f, 0.f);
		tau_comp_lpf_out  = Vector3f(0.f, 0.f, 0.f);
		return;
	}

	// -----------------------------
	// 1) tunables (DOB에서 쓰는 nominal J 그대로)
	// -----------------------------
	constexpr float Jxx = 0.0768f;
	constexpr float Jyy = 0.0871f;
	constexpr float Jzz = 0.113f;

	constexpr float invJxx = 1.0f / Jxx;
	constexpr float invJyy = 1.0f / Jyy;
	constexpr float invJzz = 1.0f / Jzz;

	// Adaptation gain Γ (DOB OFF면 더 키워도 됨)
	constexpr float Gamma_x = 8.0f;
	constexpr float Gamma_y = 8.0f;
	constexpr float Gamma_z = 8.0f;

	// L1 LPF cutoff (rad/s) (너 제한: 2 rad/s)
	constexpr float wc = 1.5f;

	// clamps
	constexpr float h_max = 5.0f;            // [N*m]
	constexpr float omega_dot_max = 2000.0f; // [rad/s^2]

	// -----------------------------
	// 2) init
	// -----------------------------
	if (!initialized) {
		omega_hat   = omega_meas_rpy;
		h_hat       = Vector3f(0.f, 0.f, 0.f);
		h_f         = Vector3f(0.f, 0.f, 0.f);
		initialized = true;
	}

	// -----------------------------
	// 3) applied torque inside predictor
	//    tau_applied = tau_cmd - h_f
	// -----------------------------
	const Vector3f tau_applied = tau_cmd_rpy - h_f;

	// -----------------------------
	// 4) predictor (rigid body rate dynamics with coriolis term)
	//    J * ω_hat_dot = τ_applied + h_hat - ω_hat × (J ω_hat)
	// -----------------------------
	const Vector3f Jomega_hat(Jxx * omega_hat(0),
	                          Jyy * omega_hat(1),
	                          Jzz * omega_hat(2));

	// const Vector3f coriolis = omega_hat.cross(Jomega_hat);

	Vector3f omega_hat_dot;
	omega_hat_dot(0) = invJxx * (tau_applied(0) + h_hat(0));//invJxx * (tau_applied(0) + h_hat(0) - coriolis(0));
	omega_hat_dot(1) = invJyy * (tau_applied(1) + h_hat(1));
	omega_hat_dot(2) = invJzz * (tau_applied(2) + h_hat(2));

	omega_hat_dot(0) = math::constrain(omega_hat_dot(0), -omega_dot_max, omega_dot_max);
	omega_hat_dot(1) = math::constrain(omega_hat_dot(1), -omega_dot_max, omega_dot_max);
	omega_hat_dot(2) = math::constrain(omega_hat_dot(2), -omega_dot_max, omega_dot_max);

	omega_hat += omega_hat_dot * dt;

	// -----------------------------
	// 5) predictor error
	// -----------------------------
	const Vector3f e_tilde = omega_meas_rpy - omega_hat;

	// -----------------------------
	// 6) adaptation law: h_hat_dot = - Γ e_tilde
	// -----------------------------
	h_hat(0) += (Gamma_x * e_tilde(0)) * dt;
	h_hat(1) += (Gamma_y * e_tilde(1)) * dt;
	h_hat(2) += (Gamma_z * e_tilde(2)) * dt;

	h_hat(0) = math::constrain(h_hat(0), -h_max, h_max);
	h_hat(1) = math::constrain(h_hat(1), -h_max, h_max);
	h_hat(2) = math::constrain(h_hat(2), -h_max, h_max);

	// -----------------------------
	// 7) L1 LPF (decoupling): h_f_dot = wc (h_hat - h_f)
	// -----------------------------
	h_f += (h_hat - h_f) * (wc * dt);

	// -----------------------------
	// 8) final torque command
	// -----------------------------
	tau_tilde_rpy = tau_cmd_rpy - h_f;

	// -----------------------------
	// 9) logging outputs
	// -----------------------------
	dhat_tau_out     = h_hat;
	tau_comp_raw_out = h_hat; // raw estimate
	tau_comp_lpf_out = h_f;   // filtered estimate used in control

	// NOTE:
	// 만약 plot에서 "compensation torque"를 실제로 baseline에 더해지는 항으로 보고 싶으면
	// tau_comp_raw_out = -h_hat; tau_comp_lpf_out = -h_f; 로 바꾸면 직관적임.
}
