#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <atomic>

#include "Body.h"
#include "PlotSim.h"
#include "DataLogger.h"
#include "Simulation.h"

int main()
{
    Body B;
    DataLogger logger;
    PlotSim plot;
    Simulation sim;

    boost::thread draw_thread(&PlotSim::draw_update, &plot, boost::ref(B), boost::ref(sim));
    boost::thread log_thread(&DataLogger::append_state, &logger, boost::ref(B), boost::ref(sim));
    boost::thread sim_thread(&Simulation::simulation_thread, &sim, boost::ref(B));
    
    sim_thread.join();
    draw_thread.join();
    log_thread.join();
    
    plot.draw_result(logger);
    
    return 0;
}
