#include "l1_adaptive_controller.hpp"

#include <mathlib/math/Functions.hpp>
#include <cmath>

using matrix::Vector3f;

namespace {

// ===================== Tunables (match your diagram) ===================== //
// Q ≈ 20 Hz  -> wc = 2π*20 rad/s
static float l1_q_fc = 90;   // rad/s

// gamma ≈ 0.1
static float l1_gamma = 0.5f;

// omega_LPF ≈ 0.01 rad/s  (1st-order LPF for final torque compensation)
static float l1_omega_lpf = 0.0005f;              // rad/s

// inertia estimate (diagonal) (same as your DOB/CoM)
static float Jxx = 0.24f;
static float Jyy = 0.27f;
static float Jzz = 0.45f;

// mass used for F_raw = m * a_body
static float mass = 8.0f;

// safety clamps (keep conservative)
static constexpr float kMaxAbsDhatTau = 40.0f;  // Nm
static constexpr float kMaxAbsCom     = 4.0f;   // m (same clamp style as your estimator)
static constexpr float kMaxAbsTauComp = 40.0f;  // Nm

static const float root2 = 1.41421356f;

// ===================== Small utilities ===================== //
static inline float constrain_with_sign(float value, float limit)
{
	if (!PX4_ISFINITE(value)) return 0.f;
	if (fabsf(value) > limit) return copysignf(limit, value);
	return value;
}

static inline void constrain_vec3(Vector3f &v, float lim)
{
	v(0) = math::constrain(v(0), -lim, lim);
	v(1) = math::constrain(v(1), -lim, lim);
	v(2) = math::constrain(v(2), -lim, lim);
}

static inline matrix::Matrix<float, 3, 3> skew(const Vector3f &a)
{
	matrix::Matrix<float, 3, 3> S;
	S(0,0)=0.f;     S(0,1)=-a(2);  S(0,2)= a(1);
	S(1,0)=a(2);    S(1,1)=0.f;    S(1,2)=-a(0);
	S(2,0)=-a(1);   S(2,1)=a(0);   S(2,2)=0.f;
	return S;
}

// ===================== 2nd-order Butterworth LPF (state-space) ===================== //
// Q(s) = wc^2 / (s^2 + sqrt(2) wc s + wc^2)
// x_dot = A x + B u,  y = C x
struct Butter2LPF {
	matrix::Matrix<float, 2, 1> x{};
	float wc{10.f};
	bool init{false};

	void set_cutoff(float wc_in, bool reset)
	{
		if (PX4_ISFINITE(wc_in) && wc_in > 0.01f) wc = wc_in;
		if (reset) { x.zero(); init = true; }
	}

	float step(float u, float dt)
	{
		if (!init) { x.zero(); init = true; }

		const float dtg = (dt > 0.f) ? math::constrain(dt, 1.25e-4f, 2.0e-2f) : 1.0e-3f;
		const float wc2 = wc * wc;

		matrix::Matrix<float, 2, 2> A;
		matrix::Matrix<float, 2, 1> B;
		matrix::Matrix<float, 1, 2> C;

		A(0,0) = -root2 * wc;  A(0,1) = -wc2;
		A(1,0) =  1.0f;        A(1,1) =  0.0f;

		B(0,0) = 1.0f;
		B(1,0) = 0.0f;

		C(0,0) = 0.0f;
		C(0,1) = wc2;

		const matrix::Matrix<float, 2, 1> x_dot = A * x + B * u;
		x += x_dot * dtg;

		const float y = (C * x)(0,0);
		return PX4_ISFINITE(y) ? y : 0.f;
	}
};

// ===================== Disturbance observation (DOB-style) states ===================== //
// Reusing exactly the idea from torque_disturbance_observer.cpp:
//
//  MinvQ: input omega -> output ~ J*sQ(omega)
//  Q:     input tau   -> output ~ Q(tau)
//  dhat_tau = MinvQ_y - Q_y

static matrix::Matrix<float, 2,2>  MinvQ_A;
static matrix::Matrix<float, 2,1>  MinvQ_B;
static matrix::Matrix<float, 1,2>  MinvQ_Cx;
static matrix::Matrix<float, 1,2>  MinvQ_Cy;
static matrix::Matrix<float, 1,2>  MinvQ_Cz;

static matrix::Matrix<float, 2,2>  Q_A;
static matrix::Matrix<float, 2,1>  Q_B;
static matrix::Matrix<float, 1,2>  Q_C;

static matrix::Matrix<float, 2,1>  MinvQ_X_x, MinvQ_X_xdot; static matrix::Matrix<float,1,1> MinvQ_X_y;
static matrix::Matrix<float, 2,1>  MinvQ_Y_x, MinvQ_Y_xdot; static matrix::Matrix<float,1,1> MinvQ_Y_y;
static matrix::Matrix<float, 2,1>  MinvQ_Z_x, MinvQ_Z_xdot; static matrix::Matrix<float,1,1> MinvQ_Z_y;

static matrix::Matrix<float, 2,1>  Q_X_x, Q_X_xdot; static matrix::Matrix<float,1,1> Q_X_y;
static matrix::Matrix<float, 2,1>  Q_Y_x, Q_Y_xdot; static matrix::Matrix<float,1,1> Q_Y_y;
static matrix::Matrix<float, 2,1>  Q_Z_x, Q_Z_xdot; static matrix::Matrix<float,1,1> Q_Z_y;

// disturbance estimate
static float dhat_tau_r = 0.f;
static float dhat_tau_p = 0.f;
static float dhat_tau_y = 0.f;

// ===================== Force Q-filter states (per-axis) ===================== //
static Butter2LPF Q_Fx, Q_Fy, Q_Fz;

// ===================== CoM estimate state ===================== //
static Vector3f com_hat{0.f, 0.f, 0.f};

// ===================== 1st-order LPF for tau_comp ===================== //
static Vector3f tau_comp_lpf{0.f, 0.f, 0.f};
static bool tau_comp_init = false;

// global init
static bool initialized = false;

} // namespace

// ===================================================================================== //

void l1_adaptive_controller(float dt,
                            const Vector3f &tau_cmd_rpy,
                            const Vector3f &omega_meas_rpy,
                            const Vector3f &lin_accel_body,
                            Vector3f &tau_tilde_rpy,
                            Vector3f &dhat_tau_out,
                            Vector3f &tau_comp_raw_out,
                            Vector3f &tau_comp_lpf_out)
{
	const float dtg = (dt > 0.f) ? math::constrain(dt, 1.25e-4f, 2.0e-2f) : 1.0e-3f;

	// ---- sanitize inputs ----
	Vector3f tau_cmd = tau_cmd_rpy;
	Vector3f omega_m = omega_meas_rpy;
	Vector3f a_b     = lin_accel_body;

	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(tau_cmd(i))) tau_cmd(i) = 0.f;
		if (!PX4_ISFINITE(omega_m(i))) omega_m(i) = 0.f;
		if (!PX4_ISFINITE(a_b(i)))     a_b(i)     = 0.f;
	}

	// ---- init once ----
	if (!initialized) {
		// Q filter matrices for DOB-style observation (same as your torque_DOB)
		const float fc = l1_q_fc;
		const float fc2 = fc * fc;

		Q_A(0,0) = -root2*fc; Q_A(0,1) = -fc2;
		Q_A(1,0) =  1.0f;     Q_A(1,1) =  0.0f;

		Q_B(0,0) = 1.0f;
		Q_B(1,0) = 0.0f;

		Q_C(0,0) = 0.0f;  Q_C(0,1) = fc2;

		MinvQ_A = Q_A;
		MinvQ_B = Q_B;

		MinvQ_Cx(0,0) = Jxx*fc2; MinvQ_Cx(0,1) = 0.f;
		MinvQ_Cy(0,0) = Jyy*fc2; MinvQ_Cy(0,1) = 0.f;
		MinvQ_Cz(0,0) = Jzz*fc2; MinvQ_Cz(0,1) = 0.f;

		// Force Q filters (same cutoff)
		Q_Fx.set_cutoff(l1_q_fc, true);
		Q_Fy.set_cutoff(l1_q_fc, true);
		Q_Fz.set_cutoff(l1_q_fc, true);

		// 1st-order LPF init
		tau_comp_lpf.zero();
		tau_comp_init = false;

		initialized = true;
	}

	// =================================================================================
	// [1] Disturbance observation: dhat_tau = MinvQs(omega) - Q(tau_cmd)
	// =================================================================================
	{
		// update matrices in case someone changes cutoff at runtime (optional)
		const float fc = l1_q_fc;
		const float fc2 = fc * fc;

		Q_A(0,0) = -root2*fc; Q_A(0,1) = -fc2;
		Q_A(1,0) =  1.0f;     Q_A(1,1) =  0.0f;

		Q_B(0,0) = 1.0f;
		Q_B(1,0) = 0.0f;

		Q_C(0,0) = 0.0f;  Q_C(0,1) = fc2;

		MinvQ_A = Q_A;
		MinvQ_B = Q_B;

		MinvQ_Cx(0,0) = Jxx*fc2; MinvQ_Cx(0,1) = 0.f;
		MinvQ_Cy(0,0) = Jyy*fc2; MinvQ_Cy(0,1) = 0.f;
		MinvQ_Cz(0,0) = Jzz*fc2; MinvQ_Cz(0,1) = 0.f;

		// Roll
		MinvQ_X_xdot = MinvQ_A * MinvQ_X_x + MinvQ_B * omega_m(0);
		MinvQ_X_x   += MinvQ_X_xdot * dtg;
		MinvQ_X_y    = MinvQ_Cx * MinvQ_X_x;

		Q_X_xdot     = Q_A * Q_X_x + Q_B * tau_cmd(0);
		Q_X_x       += Q_X_xdot * dtg;
		Q_X_y        = Q_C * Q_X_x;

		dhat_tau_r   = constrain_with_sign(MinvQ_X_y(0,0) - Q_X_y(0,0), kMaxAbsDhatTau);

		// Pitch
		MinvQ_Y_xdot = MinvQ_A * MinvQ_Y_x + MinvQ_B * omega_m(1);
		MinvQ_Y_x   += MinvQ_Y_xdot * dtg;
		MinvQ_Y_y    = MinvQ_Cy * MinvQ_Y_x;

		Q_Y_xdot     = Q_A * Q_Y_x + Q_B * tau_cmd(1);
		Q_Y_x       += Q_Y_xdot * dtg;
		Q_Y_y        = Q_C * Q_Y_x;

		dhat_tau_p   = constrain_with_sign(MinvQ_Y_y(0,0) - Q_Y_y(0,0), kMaxAbsDhatTau);

		// Yaw
		MinvQ_Z_xdot = MinvQ_A * MinvQ_Z_x + MinvQ_B * omega_m(2);
		MinvQ_Z_x   += MinvQ_Z_xdot * dtg;
		MinvQ_Z_y    = MinvQ_Cz * MinvQ_Z_x;

		Q_Z_xdot     = Q_A * Q_Z_x + Q_B * tau_cmd(2);
		Q_Z_x       += Q_Z_xdot * dtg;
		Q_Z_y        = Q_C * Q_Z_x;

		dhat_tau_y   = constrain_with_sign(MinvQ_Z_y(0,0) - Q_Z_y(0,0), kMaxAbsDhatTau);
	}

		Vector3f dhat_tau_vec(dhat_tau_r, dhat_tau_p, dhat_tau_y);

		dhat_tau_out = dhat_tau_vec;

	// =================================================================================
	// [2] Force from acceleration + Q-filtered force
	// =================================================================================
	const Vector3f F_raw = mass * a_b;

	Vector3f F_q;
	F_q(0) = Q_Fx.step(F_raw(0), dtg);
	F_q(1) = Q_Fy.step(F_raw(1), dtg);
	F_q(2) = Q_Fz.step(F_raw(2), dtg);

	// =================================================================================
	// [3] CoM adaptation: com_hat_dot = gamma * [F_q]_x * dhat_tau
	// =================================================================================
	{
		const matrix::Matrix<float, 3, 3> Fx_q = skew(F_q);
		const Vector3f com_hat_dot = l1_gamma * (Fx_q * dhat_tau_vec);

		com_hat += com_hat_dot * dtg;
		constrain_vec3(com_hat, kMaxAbsCom);
	}

	// =================================================================================
	// [4] Compensation torque: tau_comp = LPF1( com_hat × F_raw ), then tau_tilde = tau_cmd + tau_comp
	// =================================================================================

	// tau_comp_raw = F_raw × com_hat
	const matrix::Matrix<float, 3, 3> Fx_raw = skew(F_raw);
	Vector3f tau_comp_raw = Fx_raw * com_hat;   // == F_raw.cross(com_hat)
	tau_comp_raw_out = tau_comp_raw;



	// 1st order LPF using omega_lpf (rad/s): y_dot = omega*(x - y)
	// discrete (exp): y = alpha*x + (1-alpha)*y , alpha = 1 - exp(-omega*dt)
	float omega = (PX4_ISFINITE(l1_omega_lpf) && l1_omega_lpf > 0.f) ? l1_omega_lpf : 0.f;
	float alpha = 1.f;
	if (omega > 0.f) {
		alpha = 1.f - expf(-omega * dtg);
		alpha = math::constrain(alpha, 0.f, 1.f);
	}

	if (!tau_comp_init) {
		tau_comp_lpf = tau_comp_raw;
		tau_comp_init = true;
	} else {
		tau_comp_lpf = alpha * tau_comp_raw + (1.f - alpha) * tau_comp_lpf;
	}

	constrain_vec3(tau_comp_lpf, kMaxAbsTauComp);
	tau_comp_lpf_out = tau_comp_lpf;
	// final compensated torque (your instruction: "더해주면 된다")
	tau_tilde_rpy = tau_cmd + tau_comp_lpf;


}
