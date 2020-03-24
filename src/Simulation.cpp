#include "Simulation.h"
#include <iostream>

Simulation::Simulation()
{
    ddphi = 0.0f;
    
    // Controller parameters
    k_p = 1.0f;
    k_d = 2.0f;
    
    //  Simulation parameters
    sim_steps = 600
    ;
    sim_dt = 0.01;
    sim_time = 0.0f;
    sim_done = false;
}

float Simulation::get_target_acceleration(int &i)
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

float Simulation::controller(Body &B)
{
        alpha = B.get_alpha();
        alpha_dot = B.get_alpha_dot();
//        return -k_p * alpha - k_d * alpha_dot[2];
        
        beta = B.get_beta();
        beta_dot = B.get_beta_dot();
//        return -k_p * beta - k_d * beta_dot[2];

        gamma = B.get_gamma();
        gamma_dot = B.get_gamma_dot();
        return -k_p * gamma - k_d * gamma_dot[2];
        
//        return -k_p * (5*alpha + beta) - k_d * (beta_dot[2] + 5*alpha_dot[2]);
}

void Simulation::simulation_thread(Body &B)
{
    for (int i = 0; i < sim_steps; i++)
    {
        ddphi = controller(B);

        // Set accelerations
        target_acceleration = get_target_acceleration(i);
        B.set_a_psi(0.0f,target_acceleration,ddphi);
        
        std::cout << "simtime " << sim_time << std::endl;
        if (i == sim_steps - 1)
        {
            sim_done = true;
        }
        
        // Advance simulation
        B.advance(sim_dt);
        
        usleep(sim_dt*1000000);
        sim_time += sim_dt;
    }
}

float Simulation::get_sim_time()
{
    return sim_time;
}

float Simulation::get_sim_done()
{
    return sim_done;
}