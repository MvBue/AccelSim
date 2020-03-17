#pragma once
#include "Body.h"
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"
#include <Eigen/Dense>


class PlotSim
{
private:
    Gnuplot gp;
    Gnuplot gp_log;
    
    Eigen::Vector3f r_i;
    Eigen::Vector3f phi_i;
    Eigen::Vector3f v_i;
    Eigen::Vector3f omega_i;
    Eigen::Vector3f a_i;
    Eigen::Vector3f psi_i;
    
    std::vector<std::pair<double, double> > xy_pts;
//    std::vector<std::pair<double, double> > x_pts;
//    std::vector<std::pair<double, double> > y_pts;
    std::vector<std::pair<double, double> > phi_pts;
    std::vector<std::pair<double, double> > alpha_pts;
    std::vector<std::pair<double, double> > alphadot_pts;
    std::vector<std::pair<double, double> > beta_pts;
    std::vector<std::pair<double, double> > betadot_pts;
    std::vector<std::pair<double, double> > ddphi_pts;
    std::vector<std::pair<double, double> > xdot_pts;
    std::vector<std::pair<double, double> > ydot_pts;
    std::vector<std::pair<double, double> > phidot_pts;
	std::vector<boost::tuple<double, double, double, double> > heads;
	std::vector<boost::tuple<double, double, double, double> > v_pts;
	std::vector<boost::tuple<double, double, double, double> > a_pts;
	std::vector<boost::tuple<double, double, double, double> > orig_pts;
        
public:
    PlotSim();
    void append_states(int &i, float &sim_time, Body &B, float &alpha, float &alpha_dot, float &beta, 
    Eigen::Vector3f &beta_dot, Eigen::Vector3f &I_v_IB, Eigen::Vector3f &I_a_IB);
    void draw_update();
    void draw_result();
};