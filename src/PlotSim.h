#pragma once
#include "Body.h"
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"
#include <Eigen/Dense>
#include "DataLogger.h"
#include "Simulation.h"


class PlotSim
{
private:
    Gnuplot gp;
    Gnuplot gp_log;
    
    Eigen::VectorXf state;
    Eigen::MatrixXf states;
    float freeze_time = 0.0f;

//    std::vector<std::pair<double, double> > x_pts;
//    std::vector<std::pair<double, double> > y_pts;
    std::vector<std::pair<double, double> > phi_pts;
    std::vector<std::pair<double, double> > alpha_pts;
    std::vector<std::pair<double, double> > alphadot_pts;
    std::vector<std::pair<double, double> > beta_pts;
    std::vector<std::pair<double, double> > betadot_pts;
    std::vector<std::pair<double, double> > gamma_pts;
    std::vector<std::pair<double, double> > gammadot_pts;
    std::vector<std::pair<double, double> > ddphi_pts;
    std::vector<std::pair<double, double> > xdot_pts;
    std::vector<std::pair<double, double> > ydot_pts;
    std::vector<std::pair<double, double> > phidot_pts;
    
    std::vector<std::pair<double, double> > xy_pts;
	std::vector<boost::tuple<double, double, double, double> > heads;
	std::vector<boost::tuple<double, double, double, double> > v_pts;
	std::vector<boost::tuple<double, double, double, double> > a_pts;
	std::vector<boost::tuple<double, double, double, double> > orig_pts;
        
public:
    PlotSim();
    void draw_update(Body& B, Simulation &sim);
    void draw_result(DataLogger& logger);
};