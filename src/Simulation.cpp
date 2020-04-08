#include "Simulation.h"
#include "CSVReader.h"


Simulation::Simulation(std::string &filename)
{
    // Initialize variables
//    ddphi = 0.0f;
    a_target = {0.0f, 0.0f, 0.0f};
    
    // Controller parameters
    k_p = 10.0f;
    k_d = 5.0f;
    
    //  Simulation parameters
    sim_duration = 15.0f;
    sim_dt = 0.01;
    sim_time = 0.0f;
    sim_done = false;
    
    if (filename.size() > 0)
    {
        data = file_reader.get_columns(filename);
        file_reader.display_data();
        time = data.at(1).second;
        t_size = time.size();
        ax = data.at(15).second;
        ay = data.at(16).second;
        psi = data.at(10).second;
    }
    
}

void Simulation::get_target_acceleration(float &t)
{
    int i_t = 0;
    for (i_t = 0; i_t < t_size; i_t++)
    {
        if (t < time.at(i_t))
        {
            break;
        }
    }
    
//    a_target.at(0) = ax.at(i_t - 1) + (t - time.at(i_t - 1)) / (time.at(i_t) - time.at(i_t - 1)) * (ax.at(i_t) - ax.at(i_t - 1));
//    a_target.at(1) = ay.at(i_t - 1) + (t - time.at(i_t - 1)) / (time.at(i_t) - time.at(i_t - 1)) * (ay.at(i_t) - ay.at(i_t - 1));
//    a_target.at(2) = psi.at(i_t - 1) + (t - time.at(i_t - 1)) / (time.at(i_t) - time.at(i_t - 1)) * (psi.at(i_t) - psi.at(i_t - 1));
    a_target.at(0) = 0.0f;
    a_target.at(1) = 5.0f;
    a_target.at(2) = 0.0f;
    
}

float Simulation::controller(Body &B)
{
//    // Minimizes heading to velocity direction
//    alpha = B.get_alpha();
//    alpha_dot = B.get_alpha_dot();
////    return -k_p * alpha - k_d * alpha_dot[2];
//    
//    // Minimizes distance to origin
//    beta = B.get_beta();
//    beta_dot = B.get_beta_dot();
////    return -k_p * beta - k_d * beta_dot[2];
//
//    // Minimizes absolute velocity
//    gamma = B.get_gamma();
//    gamma_dot = B.get_gamma_dot();
////    return -k_p * gamma - k_d * gamma_dot[2];
//    
    r = B.get_r();
    phi = B.get_phi();
    Eigen::Vector3f a = B.get_a();
    Eigen::Vector3f a_y;
    a_y << 0.0f, a_target.at(1), 0.0f;
    Eigen::Quaternion<float> Xi_IB;
    Xi_IB = Eigen::AngleAxis<float>(phi[0], Eigen::Vector3f::UnitX());
    Eigen::Vector3f vec;
    vec = Xi_IB * (Eigen::Vector3f::UnitZ() * 9.81f + a_y);
    
    if (a.norm() == 0)
        {
            delta = 0.0f;
        } else
        {
            delta = atan2f(Eigen::Vector3f::UnitZ().cross(vec)[0],Eigen::Vector3f::UnitZ().dot(vec));
        }

    
    std::cout << delta << " " << std::endl;
        
    return (r(1)/5.0f - phi(0)) * 10.0f - (delta + B.get_omega()(0));
//    return (-delta - B.get_omega()(0)) ;
    
}

Eigen::Vector3f Simulation::gravitation_compensation(Body &B)
{
    phi = B.get_phi();
    Eigen::Quaternion<float> Xi_IB;
    Xi_IB = Eigen::AngleAxis<float>(-phi[0], Eigen::Vector3f::UnitX());
        
    return (Eigen::Vector3f::UnitZ() + Xi_IB * Eigen::Vector3f::UnitZ() * -1) * 9.81f;
}

void Simulation::simulation_thread(Body &B)
{
    B.set_r_phi(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    for (float t = 0.0f; t < sim_duration; t += sim_dt)
    {
        
        get_target_acceleration(t);
        ddphi = controller(B);
        
        grav_comp = gravitation_compensation(B);

        // Set accelerations

//        B.set_a_psi(0.0f, a_target.at(1), 0.0f, 0.0f, 0.0f, ddphi);
//        B.set_a_psi(0.0f, a_target.at(1) + grav_comp(1), grav_comp(2), ddphi, 0.0f, 0.0f);
        B.set_a_psi(0.0f, a_target.at(1) + grav_comp(1), grav_comp(2), ddphi, 0.0f, 0.0f);
        
        std::cout << "simtime " << sim_time << std::endl;
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