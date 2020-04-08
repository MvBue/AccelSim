#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <boost/tuple/tuple.hpp>

#include "Body.h"
#include "CSVReader.h"

class Simulation
{
private:
    float alpha; // angle between the heading direction and the absolute velocity vector
    Eigen::Vector3f alpha_dot;

    float beta; // angle between the acceleration vector and -I_r_BI
    Eigen::Vector3f beta_dot;
    
    float gamma; // angle between the acceleration vector and -I_v_BI
    Eigen::Vector3f gamma_dot;
    
    float delta;
    float delta_dot;
    
    Eigen::Vector3f r;
    Eigen::Vector3f phi;
    float ddphi;
    Eigen::Vector3f grav_comp;
    // Controller parameters
    float k_p;
    float k_d;
    
    //  Simulation parameters
    float sim_duration;
    float sim_dt;
    float sim_time;
    bool sim_done;
        
    CSVReader file_reader;
    std::vector<std::pair<std::string, std::vector<float>>> data;
    std::vector<float> time;
    std::vector<float> ax;
    std::vector<float> ay;
    std::vector<float> psi;
    int t_size;
    std::vector<float> a_target;
    
public:
    Simulation(std::string &filename);
    void simulation_thread(Body &B);
    float controller(Body &B);
    Eigen::Vector3f gravitation_compensation(Body &B);
    void get_target_acceleration(float &t);
    float get_sim_time();
    float get_sim_done();
};