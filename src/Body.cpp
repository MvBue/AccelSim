#include "Body.h"
#include <iostream>

Body::Body()
{
    I_r_IB << 0.0f, 0.0f, 0.0f;
    I_phi_IB << 0.0f, 0.0f, 0.0f;
    B_v_IB << 0.0f, 0.0f, 0.0f;
    I_omega_IB << 0.0f, 0.0f, 0.0f;
    B_a_IB << 0.0f, 0.0f, 0.0f;
    I_psi_IB << 0.0f, 0.0f, 0.0f;
        
    state.conservativeResize(24, 1);
    new_state = false;
    update_state_vars();
    set_state();
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
    
    state.conservativeResize(24, 1);
    new_state = false;
    update_state_vars();
    set_state();
};

void Body::display() 
{
    printf("x:%.3f, y:%.3f, phi:%.3f, vx:%.3f, vy:%.3f, omega:%.3f, ax:%.3f, ay:%.3f, psi:%.3f\n", 
    I_r_IB(0), I_r_IB(1), I_phi_IB(2), B_v_IB(0), B_v_IB(1), I_omega_IB(2), B_a_IB(0), B_a_IB(1), I_psi_IB(2));
}

void Body::advance(float dt)
{
    
    Xi_IB = Eigen::AngleAxis<float>(I_phi_IB[0], Eigen::Vector3f::UnitX());
    B_v_IB = B_v_IB + (B_a_IB + B_v_IB.cross(I_omega_IB)) * dt;
//    B_v_IB = B_v_IB + (B_a_IB) * dt;
    I_omega_IB = I_omega_IB + I_psi_IB * dt;
    I_r_IB = I_r_IB + Xi_IB * B_v_IB * dt;
    I_phi_IB = I_phi_IB + I_omega_IB * dt;
    update_state_vars();
    
    set_state();
}

void Body::update_state_vars()
{
    I_ex_B = Xi_IB * Eigen::Vector3f::UnitX();
    I_ez_B = Xi_IB * Eigen::Vector3f::UnitZ();
    I_v_IB = Xi_IB * B_v_IB;
    I_a_IB = Xi_IB * B_a_IB;
    
    alpha = atan2f (I_v_IB.cross(I_ex_B)[2],I_v_IB.dot(I_ex_B));
    
    if (I_v_IB.norm()==0)
        {
            alpha_dot = I_omega_IB;
        }
        else 
        {
            rho_instant = (I_a_IB - (I_v_IB.dot(I_a_IB) * I_v_IB / powf(I_v_IB.norm(),2))).norm() / I_v_IB.norm() * Eigen::Vector3f::UnitZ();
            alpha_dot = I_omega_IB - rho_instant;
        }

    if (I_r_IB.norm() == 0 || I_v_IB.norm() == 0)
        {
            beta = 0.0f;
        } else if (I_a_IB.norm() == 0)
        {
            beta = atan2f(-I_r_IB.cross(I_ex_B)[2],-I_r_IB.dot(I_ex_B));
        } else
        {
            beta = atan2f(-I_r_IB.cross(I_a_IB)[2],-I_r_IB.dot(I_a_IB));
        }

    if (I_r_IB.norm() == 0 || I_v_IB.norm() == 0)
        {
            beta_dot = Eigen::Vector3f::Zero();
        } else
        {
            // I_r_IB.cross(I_v_IB) = ||I_v_IB|| * ||I_r_IB|| * sin(angle_between_v_and_r)
            // AND rho_origin = ||I_v_IB|| * sin(angle_between_v_and_r) / ||I_r_IB||
            rho_origin = I_r_IB.cross(I_v_IB) / powf(I_r_IB.norm(),2);
            beta_dot = I_omega_IB - rho_origin;
        }
        
    if (I_a_IB.norm() == 0 || I_v_IB.norm() == 0)
        {
            gamma = 0.0f;
        } else
        {
            gamma = atan2f(-I_v_IB.cross(I_a_IB)[2],-I_v_IB.dot(I_a_IB));
        }
        
    if (I_a_IB.norm() == 0 || I_v_IB.norm() == 0)
        {
            gamma_dot = Eigen::Vector3f::Zero();
        } else
        {
            //rho_instant = || I_a_IB_perpToV || / || I_v_IB // * I_e_z
            rho_instant = (I_a_IB - (I_v_IB.dot(I_a_IB) * I_v_IB / powf(I_v_IB.norm(),2))).norm() / I_v_IB.norm() * Eigen::Vector3f::UnitZ();
            gamma_dot = I_omega_IB - rho_instant;
        }
        
    if (I_r_IB.norm() == 0 || I_v_IB.norm() == 0)
        {
            delta_dot = 0.0f;
        } else
        {
            delta_dot = I_omega_IB(0);
        }
}

void Body::set_r_phi(float xi = 0, float yi = 0, float zi = 0, float phi_xi = 0, float phi_yi = 0, float phi_zi = 0)
{
    I_r_IB << xi, yi, zi;
    I_phi_IB << phi_xi, phi_yi, phi_zi;
}

void Body::set_v_omega(float v_xi = 0, float v_yi = 0, float v_zi = 0, float omega_xi = 0, float omega_yi = 0, float omega_zi = 0)
{
    B_v_IB << v_xi, v_yi, v_zi;
    I_omega_IB << omega_xi, omega_yi, omega_zi;
}

void Body::set_a_psi(float a_xi = 0.0f, float a_yi = 0.0f, float a_zi = 0.0f, float psi_xi = 0.0f, float psi_yi = 0.0f, float psi_zi = 0.0f)
{
    B_a_IB << a_xi, a_yi, a_zi;
    I_psi_IB << psi_xi, psi_yi, psi_zi;
}

void Body::set_state()
{
    boost::unique_lock<boost::mutex> lock{mutex};
    state << I_r_IB[0], I_r_IB[1], I_r_IB[2], I_phi_IB[0], I_phi_IB[1], I_phi_IB[2], B_v_IB[0], B_v_IB[1], B_v_IB[2], 
    I_omega_IB[0], I_omega_IB[1], I_omega_IB[2], B_a_IB[0], B_a_IB[1], B_a_IB[2], I_psi_IB[0], I_psi_IB[1], I_psi_IB[2], 
    alpha, alpha_dot[2], beta, beta_dot[2], gamma, gamma_dot[2];
    cond.notify_all();
}

Eigen::VectorXf Body::get_state()
{
    boost::unique_lock<boost::mutex> lock{mutex};
    cond.wait(lock);
    output = state;
    return output;
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

float Body::get_alpha()
{
    return alpha;
}

Eigen::Vector3f Body::get_alpha_dot()
{
    return alpha_dot;
}

float Body::get_beta()
{
    return beta;
}

Eigen::Vector3f Body::get_beta_dot()
{
    return beta_dot;
}

float Body::get_gamma()
{
    return gamma;
}

Eigen::Vector3f Body::get_gamma_dot()
{
    return gamma_dot;
}

float Body::get_delta()
{
    return delta;
}

float Body::get_delta_dot()
{
    return delta_dot;
}
