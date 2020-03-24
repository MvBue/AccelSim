#include "DataLogger.h"

DataLogger::DataLogger()
{
    states.conservativeResize(14, 1);
}

void DataLogger::append_state(float &sim_time, Body &B, bool &sim_done)
{
    while (!sim_done)
    {
        state = B.get_state();
        states.conservativeResize(states.rows(), states.cols() + 1);
        states.col(states.cols() - 1) << sim_time, state;
    }
}

//Eigen::VectorXf DataLogger::get_latest_state()
//{
//    output = states.rightCols(1);
//    return output;
//}

Eigen::MatrixXf DataLogger::get_states()
{
    return states;
}