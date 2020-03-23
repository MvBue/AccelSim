#pragma once
#include <Eigen/Dense>
#include "Body.h"

class DataLogger 
{
private:
    Eigen::MatrixXf states;
    Eigen::VectorXf state;
public:
    DataLogger();
    void append_state(float &sim_time, Body &B);
    Eigen::VectorXf get_latest_state();
    Eigen::MatrixXf get_states();
};