/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 #include "MulticopterRateControl.hpp"

 #include <drivers/drv_hrt.h>
 #include <circuit_breaker/circuit_breaker.h>
 #include <mathlib/math/Limits.hpp>
 #include <mathlib/math/Functions.hpp>
 #include <px4_platform_common/events.h>

 #include <uORB/Publication.hpp>
 #include <uORB/topics/custom_dt.h>

 using namespace matrix;
 using namespace time_literals;
 using math::radians;

namespace {
uORB::Publication<custom_dt_s> g_custom_dt_pub{ORB_ID(custom_dt)};
}

 MulticopterRateControl::MulticopterRateControl(bool vtol) :
	 ModuleParams(nullptr),
	 WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	 _vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	 _vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	 _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
 {
	 _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	 parameters_updated();
	 _controller_status_pub.advertise();
 }

 MulticopterRateControl::~MulticopterRateControl()
 {
	 perf_free(_loop_perf);
 }

 bool
 MulticopterRateControl::init()
 {
	 if (!_vehicle_angular_velocity_sub.registerCallback()) {
		 PX4_ERR("callback registration failed");
		 return false;
	 }

	 return true;
 }

 void
 MulticopterRateControl::parameters_updated()
 {
	 // rate control parameters
	 // The controller gain K is used to convert the parallel (P + I/s + sD) form
	 // to the ideal (K * [1 + 1/sTi + sTd]) form
	 const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	 _rate_control.setPidGains(
		 rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		 rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		 rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	 gain_check = {_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get()};
	 //gain_check = {_param_mc_rollrate_d.get(), _param_mc_rollrate_d.get(), _param_mc_rollrate_d.get()};
	 _rate_control.setIntegratorLimit(
		 Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	 _rate_control.setFeedForwardGain(
		 Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	 // manual rate control acro mode rate limits
	 _acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				   radians(_param_mc_acro_y_max.get()));
 }

 void
 MulticopterRateControl::Run()
 {
	 if (should_exit()) {
		 _vehicle_angular_velocity_sub.unregisterCallback();
		 exit_and_cleanup();
		 return;
	 }
	 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	 perf_begin(_loop_perf);

	 // Check if parameters have changed
	 if (_parameter_update_sub.updated()) {
		 // clear update
		 parameter_update_s param_update;
		 _parameter_update_sub.copy(&param_update);

		 updateParams();
		 parameters_updated();
	 }

	 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	 /* run controller on gyro changes */
	 vehicle_angular_velocity_s angular_velocity;

	 if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		 const hrt_abstime now = angular_velocity.timestamp_sample;

		 // Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		 const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		 _last_run = now;


		 const Vector3f rates{angular_velocity.xyz}; // ang vel sensor streaming
		 const Vector3f angular_accel{angular_velocity.xyz_derivative}; // ang vel 미분
		 vehicle_acceleration_s acc{};
		 if (_vehicle_acceleration_sub.update(&acc)) {
		 // acc.xyz 는 Body frame 기준 m/s^2 (EKF 설정에 따라 중력 제거/포함이 다를 수 있어, 로그로 확인 추천)
		 _lin_accel_body = Vector3f{acc.xyz};
		 _accel_ts_sample = acc.timestamp_sample;
		 }
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		 /* check for custom mode ex) DOB */
		 _custom_control_mode_sub.update(&_custom_control_mode);


		 /* check for updates in other topics */
		 _vehicle_control_mode_sub.update(&_vehicle_control_mode);

		 if (_vehicle_land_detected_sub.updated()) {
			 vehicle_land_detected_s vehicle_land_detected;

			 if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				 _landed = vehicle_land_detected.landed;
				 _maybe_landed = vehicle_land_detected.maybe_landed;
			 }
		 }

		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		 _vehicle_status_sub.update(&_vehicle_status);

		 // use rates setpoint topic
		 vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		 if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			 // 각속도제어를 직접할때
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 // generate the rate setpoint from sticks
			 manual_control_setpoint_s manual_control_setpoint;

			 if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {

				 // manual rates control - ACRO mode
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

				 const Vector3f man_rate_sp{
					 math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					 math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					 math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};
					 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
					 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

				 _rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				 //_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				 _thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				 // publish rate setpoint
				 vehicle_rates_setpoint.roll = _rates_setpoint(0);
				 vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				 vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				 //_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				 vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				 _vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 }
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		 } else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 _rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				 _rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				 _rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				 _thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 }
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 }

		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ RATES CONTROL STABILIZATION ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // run the rate controller
		 //if (true/*_vehicle_control_mode.flag_control_rates_enabled*/) {
		 if(true){
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // reset integral if disarmed
			 if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				 _rate_control.resetIntegral(); // _rate_int.zero();
			 }
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 // update saturation status from control allocation feedback
			 control_allocator_status_s control_allocator_status;

			 if (_control_allocator_status_sub.update(&control_allocator_status)) {
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 Vector<bool, 3> saturation_positive;
				 Vector<bool, 3> saturation_negative;

				 if (!control_allocator_status.torque_setpoint_achieved) {
					 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
					 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
					 for (size_t i = 0; i < 3; i++) {
						 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
						 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
						 if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							 saturation_positive(i) = true;

						 } else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							 saturation_negative(i) = true;
						 }
						 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
						 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
					 }
					 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
					 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 }

				 // TODO: send the unallocated value directly for better anti-windup
				 _rate_control.setSaturationStatus(saturation_positive, saturation_negative);
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
				 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 }
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ RATES CONTROL START ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // run rate controller
			 const Vector3f att_control = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed); //return torque;

			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 // publish rate controller status
			 rate_ctrl_status_s rate_ctrl_status{};
			 _rate_control.getRateControlStatus(rate_ctrl_status);
			 rate_ctrl_status.timestamp = hrt_absolute_time();
			 _controller_status_pub.publish(rate_ctrl_status);

			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 // publish thrust and torque setpoints
			 vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			 vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			 _thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);

			 _torque_dhat.timestamp = hrt_absolute_time();

			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ Torque DOB Logic loop closing [SEUK]ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


			 desired_tau_PID_rpy(0) = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			 desired_tau_PID_rpy(1) = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			 desired_tau_PID_rpy(2) = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;


			 if(_custom_control_mode.disturbance_observer_flag)
			 {
				 torque_DOB(dt, desired_tau_PID_rpy, rates, tau_rpy_tilde, _torque_dhat, true);

				 vehicle_torque_setpoint.xyz[0] = tau_rpy_tilde(0);
				 vehicle_torque_setpoint.xyz[1] = tau_rpy_tilde(1);
				 vehicle_torque_setpoint.xyz[2] = tau_rpy_tilde(2);

			 }
			 else
			 {
				 torque_DOB(dt, desired_tau_PID_rpy, rates, tau_rpy_tilde, _torque_dhat, false);

				 vehicle_torque_setpoint.xyz[0] = desired_tau_PID_rpy(0);
				 vehicle_torque_setpoint.xyz[1] = desired_tau_PID_rpy(1);
				 vehicle_torque_setpoint.xyz[2] = desired_tau_PID_rpy(2);

			 }

			 Vector3f dhat_vec(_torque_dhat.xyz[0],
					 _torque_dhat.xyz[1],
					 _torque_dhat.xyz[2]);
			 _torque_dhat_pub.publish(_torque_dhat);

			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ CoM Estimator Logic ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

			 // center_of_mass_s center_of_mass_update;
			 center_of_mass_update.timestamp = hrt_absolute_time();



			 if(_custom_control_mode.custom_mode_flag){

				if (_custom_control_mode.trajectory_flag) dob_based_com_estimator(dt, dhat_vec, _thrust_setpoint, center_of_mass_update, _lin_accel_body, true);
				else dob_based_com_estimator(dt, dhat_vec, _thrust_setpoint, center_of_mass_update, _lin_accel_body, false);
			 
			}else{

				 matrix::Vector3f desired_tau_PID_rpy_zero{0.f, 0.f, 0.f};
				 Vector3f tau_rpy_tilde_zero{0.f, 0.f, 0.f};
				 Vector3f _thrust_setpoint_zero{0.f, 0.f, 0.f};
				dob_based_com_estimator(dt, desired_tau_PID_rpy_zero,tau_rpy_tilde_zero,center_of_mass_update, _lin_accel_body, false);
			 }

			 _center_of_mass_pub.publish(center_of_mass_update);

			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ Yaw trimming Logic ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ Yaw trimming Logic (Butterworth + dt Moving Average) ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
			// RT(고역) + TVC(저역) 분리: Butterworth(2차) 유지, fs = 1/dt 대신 이동평균(dt) 기반
			constexpr float kYawSplitFc_Hz = 0.4f;   // TVC 저역 컷오프 [Hz]
			constexpr float kYawRtMax_Nm   = 0.5f;   // RT 권한 한계 [Nm]
			constexpr float kMinFs_Hz      = 50.f;   // 보호용 최소 fs
			constexpr float kMaxFs_Hz      = 2000.f; // 보호용 최대 fs
			constexpr float kFsUpdateFrac  = 0.25f;  // Butter 계수 갱신 임계(비율)
			constexpr float kMaWindow_s    = 0.10f;  // dt 이동평균 목표 창 길이 [s] (~0.08~0.12 사이 권장)
			constexpr int   kMaMaxN        = 64;     // 이동평균 최대 샘플 수 (작게 유지)

			// 원하는 yaw torque (비정상값 가드)
			float tau_z_des = vehicle_torque_setpoint.xyz[2];
			if (!PX4_ISFINITE(tau_z_des)) {
				tau_z_des = 0.f;
			}

			if (!(dt > 0.f)) {
				// 비정상 dt면 RT만 제한, TVC는 0
				vehicle_torque_setpoint.xyz[2]   = math::constrain(tau_z_des, -kYawRtMax_Nm, +kYawRtMax_Nm);
				vehicle_torque_setpoint.yaw_trim = 0.f;

			} else {
				// ──(1) dt를 이동평균으로 스무딩 → fs = 1 / avg(dt)
				// 원형 버퍼에 최근 dt를 저장해놓고, "약 kMaWindow_s 길이"만큼의 샘플을 박스카 평균
				static float dt_hist[kMaMaxN] {};
				static int   dt_count = 0;
				static int   dt_head  = 0;  // 다음에 쓸 위치
				// 이번 스텝 dt 가드
				float dt_guard = math::constrain(dt, 1.25e-4f, 2.0e-2f);

				// 버퍼에 push
				dt_hist[dt_head] = dt_guard;
				dt_head = (dt_head + 1) % kMaMaxN;
				if (dt_count < kMaMaxN) dt_count++;

				// 목표 창 길이에 맞춰 사용할 샘플 수 K를 동적으로 결정 (8~64 사이)
				int K = (int)(kMaWindow_s / dt_guard + 0.5f);          // time-based 길이
				K = math::constrain(K, 8, kMaMaxN);
				if (K > dt_count) K = dt_count;                        // 확보된 샘플 수 이하

				// 최근 K개 dt의 합
				float sum_dt = 0.f;
				for (int i = 0; i < K; i++) {
					int idx = dt_head - 1 - i;
					if (idx < 0) idx += kMaMaxN;
					sum_dt += dt_hist[idx];
				}
				const float dt_ma = (K > 0) ? (sum_dt / (float)K) : dt_guard;

				// 이동평균으로 얻은 fs
				float fs = (dt_ma > 0.f) ? (1.f / dt_ma) : kMinFs_Hz;
				fs = math::constrain(fs, kMinFs_Hz, kMaxFs_Hz);

				// ──(2) fc를 Nyquist 여유로 클램프 (0.45 * fs 권장)
				const float fc_clamped = math::constrain(kYawSplitFc_Hz, 0.1f, 0.45f * fs);

				// ──(3) Butter2 계수 초기화/갱신 (상태는 유지)
				static float prev_fs = 0.f;
				if (!_lpf_tauz_initialized) {
					_lpf_tauz.setButter2Lowpass(fc_clamped, fs, /*reset=*/true);
					// 초기 과도 줄이기(워밍업 2~3회)
					for (int i = 0; i < 3; ++i) (void)_lpf_tauz.step(tau_z_des);
					prev_fs = fs;
					_lpf_tauz_initialized = true;

				} else {
					// 이동평균 덕분에 fs가 부드러워졌으므로, 갱신 임계도 그대로/혹은 더 타이트하게 (예: 15%) 조정 가능
					if (fabsf(fs - prev_fs) > kFsUpdateFrac * prev_fs) {
						_lpf_tauz.setButter2Lowpass(fc_clamped, fs, /*reset=*/false);
						prev_fs = fs;
					}
				}

				// ──(4) 분리: 저역(TVC), 여역(RT)
				const float tau_z_bar  = _lpf_tauz.step(tau_z_des); // TVC로 보낼 저역
				const float tau_z_fast = tau_z_des - tau_z_bar;     // RT로 보낼 고역

				// ──(5) RT 포화, 초과분은 TVC로
				const float tau_z_r = math::constrain(tau_z_fast, -kYawRtMax_Nm, +kYawRtMax_Nm);
				const float tau_z_t = tau_z_bar + (tau_z_fast - tau_z_r); // RT 포화 초과분을 TVC로

				// ──(6) 결과 배치
				vehicle_torque_setpoint.xyz[2]   = tau_z_r;  // RT(고역 + 포화 이내)
				vehicle_torque_setpoint.yaw_trim = tau_z_t;  // TVC(저역 + RT 포화 초과분)
			}



			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // _vehicle_control_mode.fl

			 vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			 vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			 _vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			 vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			 vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			 _vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);



			 updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
			 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


		 }
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		 // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	 }

	 perf_end(_loop_perf);
 }

 void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		 float dt)
 {
	 for (int i = 0; i < 3; i++) {
		 _control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	 }

	 _energy_integration_time += dt;

	 if (_energy_integration_time > 500e-3f) {

		 actuator_controls_status_s status;
		 status.timestamp = vehicle_torque_setpoint.timestamp;

		 for (int i = 0; i < 3; i++) {
			 status.control_power[i] = _control_energy[i] / _energy_integration_time;
			 _control_energy[i] = 0.f;
		 }

		 _actuator_controls_status_pub.publish(status);
		 _energy_integration_time = 0.f;
	 }
 }

 int MulticopterRateControl::task_spawn(int argc, char *argv[])
 {
	 bool vtol = false;

	 if (argc > 1) {
		 if (strcmp(argv[1], "vtol") == 0) {
			 vtol = true;
		 }
	 }

	 MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	 if (instance) {
		 _object.store(instance);
		 _task_id = task_id_is_work_queue;

		 if (instance->init()) {
			 return PX4_OK;
		 }

	 } else {
		 PX4_ERR("alloc failed");
	 }

	 delete instance;
	 _object.store(nullptr);
	 _task_id = -1;

	 return PX4_ERROR;
 }

 int MulticopterRateControl::custom_command(int argc, char *argv[])
 {
	 return print_usage("unknown command");
 }

 int MulticopterRateControl::print_usage(const char *reason)
 {
	 if (reason) {
		 PX4_WARN("%s\n", reason);
	 }

	 PRINT_MODULE_DESCRIPTION(
		 R"DESCR_STR(
 ### Description
 This implements the multicopter rate controller. It takes rate setpoints (in acro mode
 via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

 The controller has a PID loop for angular rate error.

 )DESCR_STR");

	 PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	 PRINT_MODULE_USAGE_COMMAND("start");
	 PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	 PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	 return 0;
 }

 int MulticopterRateControl::print_status()
 {
	 PX4_INFO("Running");

	 //PX4_INFO("armed_flag check : %s", _vehicle_control_mode.flag_armed ? "true" : "false" );
	 PX4_INFO("roll P : %f | pitch P : %f | yaw P : %f", (double)gain_check(0), (double)gain_check(1), (double)gain_check(2));
	 return 0;
 }

 extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
 {
	 return MulticopterRateControl::main(argc, argv);
 }
