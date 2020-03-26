#include "Simulation.h"
#include "CSVReader.h"


Simulation::Simulation(std::string &filename)
{
    ddphi = 0.0f;
    
    // Controller parameters
    k_p = 10.0f;
    k_d = 5.0f;
    
    //  Simulation parameters
    sim_duration = 30.0f;
    sim_dt = 0.01;
    sim_time = 0.0f;
    sim_done = false;
    
    if (filename.size() > 0)
    {
        data = file_reader.get_columns(filename);
        file_reader.display_data();
        time = data.at(1).second;
        t_size = time.size();
        ay = data.at(9).second;
    }
    
}

float Simulation::get_target_acceleration(float &t)
{
    int i_t = 0;
    for (i_t = 0; i_t < t_size; i_t++)
    {
        if (t < time.at(i_t))
        {
            break;
        }
    }
    
    ay_m = ay.at(i_t - 1) + (t - time.at(i_t - 1)) / (time.at(i_t) - time.at(i_t - 1)) * (ay.at(i_t) - ay.at(i_t - 1));
    
    return ay_m;
}

float Simulation::controller(Body &B)
{
    // Minimizes heading to velocity direction
    alpha = B.get_alpha();
    alpha_dot = B.get_alpha_dot();
//    return -k_p * alpha - k_d * alpha_dot[2];
    
    // Minimizes distance to origin
    beta = B.get_beta();
    beta_dot = B.get_beta_dot();
//    return -k_p * beta - k_d * beta_dot[2];

    // Minimizes absolute velocity
    gamma = B.get_gamma();
    gamma_dot = B.get_gamma_dot();
    return -k_p * gamma - k_d * gamma_dot[2];
    
    // Experimental
    // Try minimizing angle between velocity vector and inverse position vector!
//    return -k_p * (5*alpha + beta) - k_d * (beta_dot[2] + 5*alpha_dot[2]);
}

void Simulation::simulation_thread(Body &B)
{
    for (float t = 0.0f; t < sim_duration; t += sim_dt)
    {
        ddphi = controller(B);

        // Set accelerations
        target_acceleration = get_target_acceleration(t);
//        target_acceleration = 1.0f;
        B.set_a_psi(0.0f,target_acceleration,ddphi);
        
        std::cout << "simtime " << target_acceleration << std::endl;
        if (t > sim_duration - sim_dt)
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