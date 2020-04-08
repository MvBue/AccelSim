/* Convention for vectors
 * "1_variable_23"
 * position, angle, velocity or acceleration of frame 3 
 * with respect to frame 2, where 1 is the frame of reference 
 * (I: inertial frame; B = body frame)
 * */

#pragma once
#include <Eigen/Dense>
#include <boost/thread.hpp>

class Body
{
private:
    Eigen::Vector3f I_r_IB; // position vector
    Eigen::Vector3f I_phi_IB; // rotation vector
    Eigen::Vector3f B_v_IB; // local translational velocity vector
    Eigen::Vector3f I_v_IB;
    Eigen::Vector3f I_omega_IB; // rotational velocity vector
    Eigen::Vector3f B_a_IB; // local translational acceleration vector
    Eigen::Vector3f I_a_IB;
    Eigen::Vector3f I_psi_IB; // rotational velocity vector
    
    Eigen::Vector3f rho_instant; // angular velocity around instant center of rotation
    Eigen::Vector3f rho_origin; // angular velocity around origin

    float alpha; // angle between the heading direction and the absolute velocity vector
    Eigen::Vector3f alpha_dot; // derivative of alpha
    float beta; // angle between the acceleration vector and -I_r_BI
    //TODO: include derivative of acceleration vector into beta_dot
    Eigen::Vector3f beta_dot; // derivative of beta
    float gamma; // angle between the acceleration vector and -I_v_BI
    //TODO: include derivative of acceleration vector into gamma_dot
    Eigen::Vector3f gamma_dot; // derivative of gamma
    float delta; // angle between the acceleration vector and I_ez_I
    float delta_dot; // derivative of delta
    
    Eigen::Vector3f I_ex_B;
    Eigen::Vector3f I_ez_B;
    
    Eigen::Quaternion<float> Xi_IB;
    
    Eigen::VectorXf state;
    Eigen::VectorXf output;
    
    boost::mutex mutex;
    boost::condition_variable_any cond;
    bool new_state;
    
public:
    Body();
    Body(float xi, float yi, float phii, float v_xi, float v_yi, 
    float omegai, float a_xi, float a_yi, float psii);
    void display();
    void advance(float dt);
    void set_r_phi(float xi, float yi, float zi, float phi_xi, float phi_yi, float phi_zi);
    void set_v_omega(float v_xi, float v_yi, float v_zi, float omega_xi, float omega_yi, float omega_zi);
    void set_a_psi(float a_xi, float a_yi, float a_zi, float psi_xi, float psi_yi, float psi_zi);
    Eigen::Vector3f get_r();
    Eigen::Vector3f get_phi();
    Eigen::Vector3f get_v();
    Eigen::Vector3f get_omega();
    Eigen::Vector3f get_a();
    Eigen::Vector3f get_psi();
    float get_alpha();
    Eigen::Vector3f get_alpha_dot();
    float get_beta();
    Eigen::Vector3f get_beta_dot();
    float get_gamma();
    Eigen::Vector3f get_gamma_dot();
    float get_delta();
    float get_delta_dot();
    void set_state();
    Eigen::VectorXf get_state();
    void update_state_vars();
};