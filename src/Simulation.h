#pragma once
#include <Eigen/Dense>
#include <boost/tuple/tuple.hpp>

#include "Body.h"

class Simulation
{
private:
    float alpha; // angle between the heading direction and the absolute velocity vector
    Eigen::Vector3f alpha_dot;

    float beta; // angle between the acceleration vector and -I_r_BI
    Eigen::Vector3f beta_dot;
    
    float gamma; // angle between the acceleration vector and -I_v_BI
    Eigen::Vector3f gamma_dot;
    
    float ddphi;
    // Controller parameters
    float k_p;
    float k_d;
    
    //  Simulation parameters
    int sim_steps;
    float sim_dt;
    float sim_time;
    bool sim_done;
    
    float target_acceleration;
    
public:
    Simulation();
    void simulation_thread(Body &B);
    float controller(Body &B);
    float get_target_acceleration(int &i);
    float get_sim_time();
    float get_sim_done();
};