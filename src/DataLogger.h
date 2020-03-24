#pragma once
#include <Eigen/Dense>
#include "Body.h"
#include <boost/thread.hpp>

class DataLogger 
{
private:
    Eigen::MatrixXf states;
    Eigen::MatrixXf output;
    Eigen::VectorXf state;

public:
    DataLogger();
    void append_state(float &sim_time, Body &B, bool &sim_done);
//    Eigen::VectorXf get_latest_state();
    Eigen::MatrixXf get_states();
};