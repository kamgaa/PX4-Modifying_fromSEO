#pragma once

#include <uORB/topics/center_of_mass.h>
#include <matrix/matrix/math.hpp>

using matrix::Vector;

void dob_based_com_estimator(float dt, matrix::Vector3f torque_dhat, matrix::Vector3f body_force_desired, center_of_mass_s &com_log, const matrix::Vector3f &lin_accel_body, bool z_estimation_flag);


