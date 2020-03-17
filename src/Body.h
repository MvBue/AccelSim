/* Convention for vectors
 * "1_var_23"
 * position, angle, velocity or acceleration of frame 3 
 * with respect to frame 2, where 1 is the frame of reference 
 * (I: inertial frame; B = body frame)
 * */

#pragma once
#include <Eigen/Dense>


class Body
{
private:
    Eigen::Vector3f I_r_IB; // position vector
    Eigen::Vector3f I_phi_IB; // rotation vector
    Eigen::Vector3f B_v_IB; // translational velocity vector
    Eigen::Vector3f I_omega_IB; // rotational velocity vector
    Eigen::Vector3f B_a_IB; // translational acceleration vector
    Eigen::Vector3f I_psi_IB; // rotational velocity vector
    Eigen::Quaternion<float> Xi_IB;
    Eigen::Vector3f e_z;
    
public:
    Body();
    Body(float xi, float yi, float phii, float v_xi, float v_yi, 
    float omegai, float a_xi, float a_yi, float psii);
    void display();
    void advance(float dt);
    void set_r_phi(float xi, float yi, float phii);
    void set_v_omega(float v_xi, float v_yi, float omegai);
    void set_a_psi(float a_xi, float a_yi, float psi_i);
    Eigen::Vector3f get_r();
    Eigen::Vector3f get_phi();
    Eigen::Vector3f get_v();
    Eigen::Vector3f get_omega();
    Eigen::Vector3f get_a();
    Eigen::Vector3f get_psi();
};