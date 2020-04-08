#include "DataLogger.h"

DataLogger::DataLogger()
{
    states.conservativeResize(25, 1);
}

void DataLogger::append_state(Body &B, Simulation &sim)
{
    while (!sim.get_sim_done())
    {
        state = B.get_state();
        states.conservativeResize(states.rows(), states.cols() + 1);
        states.col(states.cols() - 1) << sim.get_sim_time(), state;
    }
}

Eigen::MatrixXf DataLogger::get_states()
{
    return states;
}