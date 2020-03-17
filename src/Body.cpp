#include "Body.h"


Body::Body()
{
    I_r_IB << 0.0f, 0.0f, 0.0f;
    I_phi_IB << 0.0f, 0.0f, 0.0f;
    B_v_IB << 0.0f, 0.0f, 0.0f;
    I_omega_IB << 0.0f, 0.0f, 0.0f;
    B_a_IB << 0.0f, 0.0f, 0.0f;
    I_psi_IB << 0.0f, 0.0f, 0.0f;

    //Declare constants
    e_z << 0.0f, 0.0f, 1.0f;
}

Body::Body(float xi = 0.0f, float yi = 0.0f, float phii = 0.0f, float v_xi = 0.0f, float v_yi = 0.0f, 
    float omegai = 0.0f, float a_xi = 0.0f, float a_yi = 0.0f, float psii = 0.0f)
{
    I_r_IB << xi, yi, 0.0f;
    I_phi_IB << 0.0f, 0.0f, phii;
    B_v_IB << v_xi, v_yi, 0.0f;
    I_omega_IB << 0.0f, 0.0f, omegai;
    B_a_IB << a_xi, a_yi, 0.0f;
    I_psi_IB << 0.0f, 0.0f, psii;
};

void Body::display() 
{
    printf("x:%.3f, y:%.3f, phi:%.3f, vx:%.3f, vy:%.3f, omega:%.3f, ax:%.3f, ay:%.3f, psi:%.3f\n", 
    I_r_IB(0), I_r_IB(1), I_phi_IB(2), B_v_IB(0), B_v_IB(1), I_omega_IB(2), B_a_IB(0), B_a_IB(1), I_psi_IB(2));
}

void Body::advance(float dt)
{
    Xi_IB = Eigen::AngleAxis<float>(I_phi_IB[2], e_z);
    B_v_IB = B_v_IB + (B_a_IB + B_v_IB.cross(I_omega_IB)) * dt;
    I_omega_IB = I_omega_IB + I_psi_IB * dt;
    I_r_IB = I_r_IB + Xi_IB * B_v_IB * dt;
    I_phi_IB = I_phi_IB + I_omega_IB * dt;
    
}

void Body::set_r_phi(float xi = 0, float yi = 0, float phii = 0)
{
    I_r_IB << xi, yi, 0.0f;
    I_phi_IB << 0.0f, 0.0f, phii;
}

void Body::set_v_omega(float v_xi = 0, float v_yi = 0, float omegai = 0)
{
    B_v_IB << v_xi, v_yi, 0.0f;
    I_omega_IB << 0.0f, 0.0f, omegai;
}

void Body::set_a_psi(float a_xi = 0, float a_yi = 0, float psii = 0)
{
    B_a_IB << a_xi, a_yi, 0.0f;
    I_psi_IB << 0.0f, 0.0f, psii;
}

Eigen::Vector3f Body::get_r()
{
    return I_r_IB;
}

Eigen::Vector3f Body::get_phi()
{
    return I_phi_IB;
}

Eigen::Vector3f Body::get_v()
{
    return B_v_IB;
}

Eigen::Vector3f Body::get_omega()
{
    return I_omega_IB;
}

Eigen::Vector3f Body::get_a()
{
    return B_a_IB;
}

Eigen::Vector3f Body::get_psi()
{
    return I_psi_IB;
}
