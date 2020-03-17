#include <iostream>
#include <Eigen/Dense>
#include <boost/thread.hpp>

#include "Body.h"
#include "PlotSim.h"

float ddy;

float get_ddy(int &i)
{
    float y;
    if (i < 500)
    {
        y = i * 10.0f / 500.0f;
    }else if (i < 1000)
    {
        y = 10.0f - (i - 500) * 10.0f / 500.0f;
    }else
    {
        y = 0.0f;
    }
    return y;
}

void simulation_thread(Body &B)
{
    PlotSim plot;
    
    float alpha;
    float alpha_dot;
    float beta;
    Eigen::Vector3f beta_dot;
    
//  Set variables
    float ddphi = 0.0f;
//    float k_p = 5.0f; //use with alpha
//    float k_d = 3.0f; //use with alpha
    float k_p = 10.0f;
    float k_d = 15.0f;
    
    //  Start simulation
    int sim_steps = 1501;
    float sim_dt = 0.01;
    float sim_time = 0;
    
    Eigen::Vector3f I_r_IB;
    Eigen::Vector3f I_r_IB_perp;
    Eigen::Vector3f I_phi_IB;
    Eigen::Vector3f B_v_IB;
    Eigen::Vector3f I_v_IB;
    Eigen::Vector3f B_a_IB;
    Eigen::Vector3f I_omega_IB;
    Eigen::Vector3f I_a_IB;
    Eigen::Vector3f B_ex_B(1,0,0); // x-Axis unit vector in frame B
    Eigen::Vector3f I_ex_B;
    Eigen::Quaternion<float> Xi_IB; // rotation transformantion from frame B to I
    Eigen::Quaternion<float> Xi_90z; // 90 degree rotation around z-Axis
    
    // constants
    Eigen::Vector3f e_z(0,0,1);
    Xi_90z = Eigen::AngleAxis<float>(M_PI/2.0, e_z);
    
    for (int i = 0; i < sim_steps; i++)
    {
//        std::cout << sim_time << std::endl;
        
        I_r_IB = B.get_r();
        I_phi_IB = B.get_phi();
        B_v_IB = B.get_v();
        I_omega_IB = B.get_omega();
        B_a_IB = B.get_a();
        Xi_IB = Eigen::AngleAxis<float>(I_phi_IB[2], e_z);
        I_a_IB = Xi_IB * B_a_IB;
        I_v_IB = Xi_IB * B_v_IB;
        I_ex_B = Xi_IB * B_ex_B;
        I_r_IB_perp = Xi_90z * I_r_IB / I_r_IB.norm();
                
//        std::cout << sim_time << "  " << beta << "  " << beta_dot[2] << std::endl;
//        std::cout << sim_time << "  " << alpha << "  " << alpha_dot << std::endl;
//        std::cout << "x" << I_v_IB.norm() << std::endl;
//        std::cout << "v" << (I_v_IB.dot(e_x_T) * e_x_T).norm() << std::endl;
        
        // angle between the heading direction and the absolute velocity vector
        alpha = atan2f(I_ex_B.cross(I_v_IB)[2],I_ex_B.dot(I_v_IB));
        
        if (I_v_IB.norm()==0)
        {
            alpha_dot = I_omega_IB[2];
        }
        else 
        {
            alpha_dot = I_omega_IB[2] - get_ddy(i) * cos(alpha) / I_v_IB.norm();
            // a_z = ddy * cos(alpha); omega = a_z / (sqrtf(pow(x_dot_i[0],2) + pow(x_dot_i[1],2))); 
            //alpha_dot = I_omega_IB[2] - omega;
        }
        
        // angle between the acceleration vector and -I_r_BI
        if (I_r_IB.norm() == 0 || I_v_IB.norm() == 0)
        {
            beta = 0;
            beta_dot << 0, 0, 0;
        } else if (I_a_IB.norm() == 0)
        {
            beta = atan2f(I_ex_B.cross(-I_r_IB)[2],I_ex_B.dot(-I_r_IB));;
            beta_dot = I_r_IB.cross(I_r_IB_perp.dot(I_v_IB) * I_r_IB_perp) / powf(I_r_IB.norm(),2) - I_omega_IB;
        } else
        {
            beta = atan2f(I_a_IB.cross(-I_r_IB)[2],I_a_IB.dot(-I_r_IB));
            
            // omega_circle = r cross v / (||r||)^2
            beta_dot = I_r_IB.cross(I_r_IB_perp.dot(I_v_IB) * I_r_IB_perp) / powf(I_r_IB.norm(),2) - I_omega_IB;
        }
        
        
//        ddphi = k_p * alpha - k_d * alpha_dot;
        ddphi = k_p * beta + k_d * beta_dot[2];
//        ddphi = k_p * (5*alpha + beta) + k_d * (beta_dot[2] - 5*alpha_dot);
        float temp = 0.0f;
        ddy = get_ddy(i);
        B.set_a_psi(temp,ddy,ddphi);
        
        std::cout << sim_time << "  " << beta_dot << std::endl;
        
        B.advance(sim_dt);
        sim_time += sim_dt;
        
        plot.append_states(i, sim_time, B, alpha, alpha_dot, beta, beta_dot, I_v_IB, I_a_IB);
        plot.draw_update();
        
        usleep(sim_dt*1000000);
    }
    plot.draw_result();
}

int main()
{
    Body B;
    B.set_v_omega(0,0,0);
    int temp = 0;
    ddy = get_ddy(temp);
    B.set_a_psi(0,ddy,0);
    
    simulation_thread(B);
    
    return 0;
}
