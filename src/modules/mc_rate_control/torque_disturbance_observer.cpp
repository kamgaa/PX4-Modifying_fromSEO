
#include "torque_disturbance_observer.hpp"
#include <matrix/matrix/math.hpp>
#include <cmath>


using matrix::Vector;


namespace {   // ← 전역 상태 캡슐화 시작

// ============ torque DOB 상태 변수들 ============ //
matrix::Matrix<float, 2,2>  MinvQ_T_A;
matrix::Matrix<float, 2,1>  MinvQ_T_B;

matrix::Matrix<float, 1,2>  MinvQ_T_C_x;
matrix::Matrix<float, 1,2>  MinvQ_T_C_y;
matrix::Matrix<float, 1,2>  MinvQ_T_C_z;

matrix::Matrix<float, 2,2>  Q_T_A;
matrix::Matrix<float, 2,1>  Q_T_B;
matrix::Matrix<float, 1,2>  Q_T_C;

matrix::Matrix<float, 2,1>  MinvQ_T_X_x;
matrix::Matrix<float, 2,1>  MinvQ_T_X_x_dot;
matrix::Matrix<float, 1,1>  MinvQ_T_X_y;

matrix::Matrix<float, 2,1>  Q_T_X_x;
matrix::Matrix<float, 2,1>  Q_T_X_x_dot;
matrix::Matrix<float, 1,1>  Q_T_X_y;

matrix::Matrix<float, 2,1>  MinvQ_T_Y_x;
matrix::Matrix<float, 2,1>  MinvQ_T_Y_x_dot;
matrix::Matrix<float, 1,1>  MinvQ_T_Y_y;

matrix::Matrix<float, 2,1>  Q_T_Y_x;
matrix::Matrix<float, 2,1>  Q_T_Y_x_dot;
matrix::Matrix<float, 1,1>  Q_T_Y_y;

matrix::Matrix<float, 2,1>  MinvQ_T_Z_x;
matrix::Matrix<float, 2,1>  MinvQ_T_Z_x_dot;
matrix::Matrix<float, 1,1>  MinvQ_T_Z_y;

matrix::Matrix<float, 2,1>  Q_T_Z_x;
matrix::Matrix<float, 2,1>  Q_T_Z_x_dot;
matrix::Matrix<float, 1,1>  Q_T_Z_y;

// ============ 파라미터/상태 ============ //
float torque_dob_fc = 3.0f; // radian임.!!     6rad = 1 hz임
float dhat_tau_r = 0.f;
float dhat_tau_p = 0.f;
float dhat_tau_y = 0.f;

float root2 = sqrtf(2.0f); // damping factor = 0.707

// 관성 추정값
float Jxx = 0.15f; //
float Jyy = 0.15f;
float Jzz = 0.20f;

// ============ 내부 유틸 ============ //
float constrain_with_sign(float value, float limit = 10.0f)
{
    if (!std::isfinite(value)) {
        return 0.f;
    }
    if (fabsf(value) > limit) {
        return copysignf(limit, value);
    }
    return value;
}

} // namespace   ← 전역 상태 캡슐화 끝



// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void torque_DOB(float dt, matrix::Vector3f tau_rpy_desired, matrix::Vector3f imu_omega, matrix::Vector3f &tau_rpy_tilde, torque_dhat_s &torque_dhat, bool compensate_flag)
{

  // Desired torque inputs

  float tau_r=tau_rpy_desired(0);
  float tau_p=tau_rpy_desired(1);
  float tau_y=tau_rpy_desired(2);

  float fc2 = torque_dob_fc * torque_dob_fc;



  //========================[1] Q-Filter Definition (Same for all axes)========================//
  Q_T_A(0,0) = -root2*torque_dob_fc; Q_T_A(0,1) = -fc2;
  Q_T_A(1,0) =  1.0f;                Q_T_A(1,1) = 0.0f;

  Q_T_B(0,0) = 1.0f;
  Q_T_B(1,0) = 0.0f;

  Q_T_C(0,0) = 0.0f;       Q_T_C(0,1) = fc2;

  //========================[2] M^-1 Q Filter Definition (Different per axis)===================//
  MinvQ_T_A(0,0) = -root2*torque_dob_fc; MinvQ_T_A(0,1) = -fc2;
  MinvQ_T_A(1,0) = 1.0f;                 MinvQ_T_A(1,1) = 0.f;

  MinvQ_T_B(0,0) = 1.0f;
  MinvQ_T_B(1,0) = 0.f;

  MinvQ_T_C_x(0,0) = Jxx*fc2; MinvQ_T_C_x(0,1) = 0.f;
  MinvQ_T_C_y(0,0) = Jyy*fc2; MinvQ_T_C_y(0,1) = 0.f;
  MinvQ_T_C_z(0,0) = Jzz*fc2; MinvQ_T_C_z(0,1) = 0.f;

  //========================[3] Compensation Terms (SEUK!!)====//
  float tautilde_r = compensate_flag ? (tau_r - dhat_tau_r) : tau_r;
  float tautilde_p = compensate_flag ? (tau_p - dhat_tau_p) : tau_p;
  float tautilde_y = compensate_flag ? (tau_y - dhat_tau_y) : tau_y;


  //========================[3] Roll Axis DoB===============================================//

  MinvQ_T_X_x_dot = MinvQ_T_A*MinvQ_T_X_x+MinvQ_T_B*imu_omega(0);
  MinvQ_T_X_x    += MinvQ_T_X_x_dot* dt;
  MinvQ_T_X_y     = MinvQ_T_C_x*MinvQ_T_X_x;

  Q_T_X_x_dot     = Q_T_A*Q_T_X_x+Q_T_B*tautilde_r;
  Q_T_X_x        += Q_T_X_x_dot*dt;
  Q_T_X_y         = Q_T_C*Q_T_X_x;

  dhat_tau_r      = constrain_with_sign(MinvQ_T_X_y(0,0) - Q_T_X_y(0,0));

  //========================[4] Pitch Axis DoB==============================================//

  MinvQ_T_Y_x_dot = MinvQ_T_A*MinvQ_T_Y_x+MinvQ_T_B*imu_omega(1);
  MinvQ_T_Y_x    += MinvQ_T_Y_x_dot*dt;
  MinvQ_T_Y_y     = MinvQ_T_C_y*MinvQ_T_Y_x;

  Q_T_Y_x_dot     = Q_T_A*Q_T_Y_x+Q_T_B*tautilde_p;
  Q_T_Y_x        += Q_T_Y_x_dot*dt;
  Q_T_Y_y         = Q_T_C*Q_T_Y_x;

  dhat_tau_p      = constrain_with_sign(MinvQ_T_Y_y(0,0) - Q_T_Y_y(0,0)); // 나중에 수정

  //========================[5] Yaw Axis DoB===============================================//

  MinvQ_T_Z_x_dot = MinvQ_T_A*MinvQ_T_Z_x+MinvQ_T_B*imu_omega(2);
  MinvQ_T_Z_x    += MinvQ_T_Z_x_dot*dt;
  MinvQ_T_Z_y     = MinvQ_T_C_z*MinvQ_T_Z_x;

  Q_T_Z_x_dot     = Q_T_A*Q_T_Z_x+Q_T_B*tautilde_y;
  Q_T_Z_x        += Q_T_Z_x_dot*dt;
  Q_T_Z_y         = Q_T_C*Q_T_Z_x;

  dhat_tau_y      = constrain_with_sign(MinvQ_T_Z_y(0,0) - Q_T_Z_y(0,0));

  //========================[6] Estimate Error and Torque Compensation=======================//



  tau_rpy_tilde(0) = tau_r - dhat_tau_r;
  tau_rpy_tilde(1) = tau_p - dhat_tau_p;
  tau_rpy_tilde(2) = tau_y - dhat_tau_y;

  torque_dhat.xyz[0] = dhat_tau_r;
  torque_dhat.xyz[1] = dhat_tau_p;
  torque_dhat.xyz[2] = dhat_tau_y;

}
