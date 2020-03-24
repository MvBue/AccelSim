#pragma once
#include <Eigen/Dense>
#include "Body.h"
#include <boost/thread.hpp>
#include "Simulation.h"

class DataLogger 
{
private:
    Eigen::MatrixXf states;
    Eigen::MatrixXf output;
    Eigen::VectorXf state;

public:
    DataLogger();
    void append_state(Body &B, Simulation &sim);
    Eigen::MatrixXf get_states();
};