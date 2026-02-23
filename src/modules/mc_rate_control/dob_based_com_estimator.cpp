#include "dob_based_com_estimator.hpp"
#include <matrix/matrix/math.hpp>
#include <mathlib/math/Functions.hpp> // math::constrain
#include <cmath>

// 내부 전역 상태를 캡슐화해서 ODR 충돌 방지
namespace {

// 천천히 수렴하는 필터 상태
matrix::Vector3f com_update{};
matrix::Vector3f com_update_lpf{}; // LPF 적용 후 값

// LPF 계수 (2초 정도 시간 상수)
constexpr float tau = 0.15f;

// Q-filter 및 계산에 쓰이는 상태 변수들
matrix::Matrix<float, 3,3> b_F_X_mat; // [F]x

matrix::Matrix<float, 2,2>  Q_T_A_est;
matrix::Matrix<float, 2,1>  Q_T_B_est;
matrix::Matrix<float, 1,2>  Q_T_C_est;

matrix::Matrix<float, 2,1>  Q_T_X_x_est;
matrix::Matrix<float, 2,1>  Q_T_X_x_dot_est;
matrix::Matrix<float, 1,1>  Q_T_X_y_est;

matrix::Matrix<float, 2,1>  Q_T_Y_x_est;
matrix::Matrix<float, 2,1>  Q_T_Y_x_dot_est;
matrix::Matrix<float, 1,1>  Q_T_Y_y_est;

matrix::Matrix<float, 2,1>  Q_T_Z_x_est;
matrix::Matrix<float, 2,1>  Q_T_Z_x_dot_est;
matrix::Matrix<float, 1,1>  Q_T_Z_y_est;

matrix::Matrix<float,3,3> A;
matrix::Vector3f b_F_LPF;

matrix::Vector3f com_hat_dot;



// 파라미터
float torque_dob_fc_est = 1.0f;   // [rad]
float est_gamma   = 4.0e-6f;//6.0e-6f;   // x,y
float est_gamma_z = 2.0e-4f;//8.5e-5f;   // z

float Jxx_est = 0.0768;
float Jyy_est = 0.0871;
float Jzz_est = 0.113;
// static float mass = 8.0f;

// // === [추가] 축별 변화율 제한(절대값, 단위: m/s) ===
// float com_rate_lim_x = 1000000.0f;
// float com_rate_lim_y = 1000000.0f;
// float com_rate_lim_z = 1000000.0f;
// // 내부 전용 유틸
// void saturate_vector3_by_limits(matrix::Vector3f &v,
//                                 float lim_x, float lim_y, float lim_z)
// {
//     v(0) = math::constrain(v(0), -lim_x, lim_x);
//     v(1) = math::constrain(v(1), -lim_y, lim_y);
//     v(2) = math::constrain(v(2), -lim_z, lim_z);
// }


// 내부 전용 유틸
void constrain_vector3_components(matrix::Vector3f &v)
{
    v(0) = math::constrain(v(0), -1.0f, 1.0f);
    v(1) = math::constrain(v(1), -1.0f, 1.0f);
    v(2) = math::constrain(v(2), -1.0f, 1.0f);
}

} // namespace


void dob_based_com_estimator(float dt, matrix::Vector3f torque_dhat, matrix::Vector3f body_force_desired, center_of_mass_s &com_log, const matrix::Vector3f &lin_accel_body, bool z_estimation_flag)
{

    const float root2 = 1.41421356f;
    const float fc2 = torque_dob_fc_est * torque_dob_fc_est;

    // body_force_desired = mass * lin_accel_body;    // 힘 말고 가속도 쓸거면 이거 해라. 그냥 힘쓸거면 주석쳐라

    //========================[1] Q-Filter Definition (Same for all axes)========================//
    Q_T_A_est(0,0) = -root2 * torque_dob_fc_est; Q_T_A_est(0,1) = -fc2;
    Q_T_A_est(1,0) = 1.0f;                        Q_T_A_est(1,1) = 0.0f;

    Q_T_B_est(0,0) = 1.0f;
    Q_T_B_est(1,0) = 0.0f;

    Q_T_C_est(0,0) = 0.0f;       Q_T_C_est(0,1) = fc2;

    //========================[2] X Axis Force===============================================//

    Q_T_X_x_dot_est     = Q_T_A_est*Q_T_X_x_est+Q_T_B_est*body_force_desired(0);
    Q_T_X_x_est        += Q_T_X_x_dot_est*dt;
    Q_T_X_y_est         = Q_T_C_est*Q_T_X_x_est;

    b_F_LPF(0) = Q_T_X_y_est(0,0);

    //========================[3] Y Axis Force===============================================//

    Q_T_Y_x_dot_est     = Q_T_A_est*Q_T_Y_x_est+Q_T_B_est*body_force_desired(1);
    Q_T_Y_x_est        += Q_T_Y_x_dot_est*dt;
    Q_T_Y_y_est         = Q_T_C_est*Q_T_Y_x_est;

    b_F_LPF(1) = Q_T_Y_y_est(0,0);

    //========================[4] Z Axis Force===============================================//

    Q_T_Z_x_dot_est     = Q_T_A_est*Q_T_Z_x_est+Q_T_B_est*body_force_desired(2);
    Q_T_Z_x_est        += Q_T_Z_x_dot_est*dt;
    Q_T_Z_y_est         = Q_T_C_est*Q_T_Z_x_est;

    b_F_LPF(2) = Q_T_Z_y_est(0,0);


    //========================[5] Force LPF Cross Product Matrix ===============================================//
    b_F_X_mat(0,0) = 0.f;          b_F_X_mat(0,1) = -b_F_LPF(2);    b_F_X_mat(0,2) = b_F_LPF(1);
    b_F_X_mat(1,0) = b_F_LPF(2);   b_F_X_mat(1,1) = 0.f;            b_F_X_mat(1,2) =-b_F_LPF(0);
    b_F_X_mat(2,0) =-b_F_LPF(1);   b_F_X_mat(2,1) = b_F_LPF(0);     b_F_X_mat(2,2) = 0.f;

    //========================[6] Definition A transpose ========================================//

    matrix::Matrix<float,3,3> MoI_inv;

    MoI_inv(0,0)=1.0f/Jxx_est; MoI_inv(0,1)=0.f;          MoI_inv(0,2)=0.f;
    MoI_inv(1,0)=0.f;          MoI_inv(1,1)=1.0f/Jyy_est; MoI_inv(1,2)=0.f;
    MoI_inv(2,0)=0.f;          MoI_inv(2,1)=0.f;          MoI_inv(2,2)=1.0f/Jzz_est;

    A = MoI_inv * b_F_X_mat;

    //========================[7] estimated com integration ========================================//

    com_hat_dot = A.transpose() * torque_dhat;


    // === 적분 ===
    com_update(0) += (est_gamma   * com_hat_dot(0)) * dt;
    com_update(1) += (est_gamma   * com_hat_dot(1)) * dt;
    if (z_estimation_flag) com_update(2) += (est_gamma_z * com_hat_dot(2)) * dt;




    constrain_vector3_components(com_update);

    static bool lpf_init = false;
    if (!lpf_init) {
        com_update_lpf = com_update; // 초기화
        lpf_init = true;
    }


    // 목표 시간상수 tau [s] 지정 (예: 2초)
    float alpha = 0.0f;
    if (dt > 0.0f && tau > 0.0f) {
        alpha = 1.0f - expf(-dt / tau);
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
    }

    com_update_lpf = alpha * com_update + (1.f - alpha) * com_update_lpf;
    constrain_vector3_components(com_update_lpf);



    com_log.com_lpf[0] = com_update_lpf(0); // after filtering
    com_log.com_lpf[1] = com_update_lpf(1);
    com_log.com_lpf[2] = com_update_lpf(2);




    com_log.com_update[0] = com_update(0); // for update on control allocator matrix
    com_log.com_update[1] = com_update(1);
    com_log.com_update[2] = com_update(2);


}
