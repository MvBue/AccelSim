#include <iostream>
#include <Eigen/Dense>
#include <boost/thread.hpp>

#include "Body.h"
#include "PlotSim.h"
#include "DataLogger.h"

float target_acceleration;

float get_target_acceleration(int &i)
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
    // Initialize variables
    PlotSim plot;
    DataLogger logger;
    
    float alpha; // angle between the heading direction and the absolute velocity vector
    Eigen::Vector3f alpha_dot;

    float beta; // angle between the acceleration vector and -I_r_BI
    Eigen::Vector3f beta_dot;
    float ddphi = 0.0f;
    
    // Controller parameters
    float k_p = 1.0f;
    float k_d = 1.0f;
    
    //  Simulation parameters
    int sim_steps = 1501;
    float sim_dt = 0.01;
    float sim_time = 0;
    
    for (int i = 0; i < sim_steps; i++)
    {
        
        // Controller
        alpha = B.get_alpha();
        alpha_dot = B.get_alpha_dot();
//        ddphi = -k_p * alpha - k_d * alpha_dot[2];
        
        beta = B.get_beta();
        beta_dot = B.get_beta_dot();
//        ddphi = -k_p * beta - k_d * beta_dot[2];
        
        ddphi = -k_p * (5*alpha + beta) - k_d * (beta_dot[2] + 5*alpha_dot[2]);

        // Set accelerations
        target_acceleration = get_target_acceleration(i);
        B.set_a_psi(0.0f,target_acceleration,ddphi);
        
        // Advance simulation
        B.advance(sim_dt);
        sim_time += sim_dt;
        
        // Log State
        logger.append_state(sim_time, B);
        
        // Draw live plot
        plot.draw_update(logger);
        
        std::cout << "sim_time: " << sim_time << std::endl;
        
        usleep(sim_dt*1000000);
    }
    plot.draw_result(logger);
}

int main()
{
    Body B;

    simulation_thread(B);
    
    return 0;
}
