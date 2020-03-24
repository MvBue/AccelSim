#include <iostream>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <atomic>

#include "Body.h"
#include "PlotSim.h"
#include "DataLogger.h"

Body B;
DataLogger logger;
PlotSim plot;
float target_acceleration;
float sim_time = 0.0f;
bool sim_done = false;

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

void simulation_thread()
{
    // Initialize variables
    
    
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
        std::cout << "simtime " << sim_time << std::endl;
        if (i == sim_steps - 1)
        {
            sim_done = true;
        }
        
        B.advance(sim_dt);
        
        usleep(sim_dt*1000000);
        sim_time += sim_dt;
    }
    plot.draw_result(logger);
}

int main()
{
    boost::thread draw_thread(&PlotSim::draw_update, &plot, boost::ref(logger), boost::ref(B), boost::ref(sim_time), boost::ref(sim_done));
    boost::thread log_thread(&DataLogger::append_state, &logger, boost::ref(sim_time), boost::ref(B), boost::ref(sim_done));
    boost::thread sim_thread(simulation_thread);
    
    sim_thread.join();
    draw_thread.join();
    log_thread.join();
    
    return 0;
}
