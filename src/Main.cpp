#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <boost/tuple/tuple.hpp>

#include <vector>
#include <utility>
#include "gnuplot-iostream.h"

#include "Body.h"

using namespace Eigen;

int main()
{
    std::vector<std::pair<double, double> > xy_pts;
//    std::vector<std::pair<double, double> > x_pts;
//    std::vector<std::pair<double, double> > y_pts;
    std::vector<std::pair<double, double> > phi_pts;
    std::vector<std::pair<double, double> > alpha_pts;
    std::vector<std::pair<double, double> > alphadot_pts;
    std::vector<std::pair<double, double> > ddphi_pts;
    std::vector<std::pair<double, double> > xdot_pts;
    std::vector<std::pair<double, double> > ydot_pts;
    std::vector<std::pair<double, double> > phidot_pts;
	std::vector<boost::tuple<double, double, double, double> > heads;
    float ddphi = 0.0f;
    float k_p = 100.0f;
    float k_d = 20.0f;
    float alpha;
    float alpha_dot;
    float ddy = 10.0f;
    float v_abs;
    
    Body B;
    B.set_x_dot(0,0,0);
    B.set_x_ddot(0,ddy,0);
    B.display();
        
//  Start simulation
    int sim_steps = 500;
    float sim_dt = 0.01;
    float sim_time = 0;
    
    Vector3f x_i = B.get_x();
    Vector3f x_dot_i = B.get_x_dot();
    
    Gnuplot gp;
    Gnuplot gp_log;
    gp << "set size ratio -1\n";
    
    for (int i = 0; i<sim_steps; i++)
    {
        x_i = B.get_x();
        x_dot_i = B.get_x_dot();
        alpha = atan2f(x_dot_i[1],x_dot_i[0]);
        v_abs = sqrtf(pow(x_dot_i[0],2) + pow(x_dot_i[1],2));
        if (v_abs==0)
        {
            alpha_dot = x_dot_i[2];
        }
        else 
        {
            alpha_dot = x_dot_i[2] - ddy * cos(alpha) / v_abs;
        }
//        a_z = ddy * cos(alpha);
//        omega = a_z / (sqrtf(pow(x_dot_i[0],2) + pow(x_dot_i[1],2)));
//        alpha_dot = x_dot_i[2] - omega;
        
        ddphi = k_p * alpha - k_d * alpha_dot;
        std::cout << sim_time << std::endl;
        B.set_x_ddot(0,ddy,ddphi);
        
        xy_pts.push_back(std::make_pair(x_i[0], x_i[1]));
//        x_pts.push_back(std::make_pair(sim_time, x_i[0]));
//        y_pts.push_back(std::make_pair(sim_time, x_i[1]));
        phi_pts.push_back(std::make_pair(sim_time, x_i[2]));
        alpha_pts.push_back(std::make_pair(sim_time, alpha));
        alphadot_pts.push_back(std::make_pair(sim_time, alpha_dot));
        ddphi_pts.push_back(std::make_pair(sim_time, ddphi));
        xdot_pts.push_back(std::make_pair(sim_time, x_dot_i[0]));
        ydot_pts.push_back(std::make_pair(sim_time, x_dot_i[1]));
        phidot_pts.push_back(std::make_pair(sim_time, x_dot_i[2]));
        
        if (i%50==0)
        {
            heads.push_back(boost::make_tuple(x_i[0],x_i[1], cos(x_i[2]), sin(x_i[2])));
        }
        
        gp << "plot '-' binary" << gp.binFmt1d(xy_pts, "record") << "with lines title 'xy', '-' with vectors title 'heading'\n";
        gp.sendBinary1d(xy_pts);
        gp.send1d(heads);

        gp.flush();
        
        B.advance(sim_dt);
        sim_time += sim_dt;
        
        usleep(sim_dt*1000000);
    }
    
    gp_log << "set multiplot layout 1,2\n";
    gp_log << "plot '-' with lines title 'alpha', '-' with lines title 'alpha_{dot}', '-' with lines title 'ddphi'\n";
    gp_log.send1d(alpha_pts);
    gp_log.send1d(alphadot_pts);
    gp_log.send1d(ddphi_pts);
//    gp_log.send1d(phi_pts);
//    gp_log << "plot '-' with lines title 'alpha_{dot}', '-' with lines title 'omega', '-' with lines title 'phi_dot'\n";
//    gp_log.send1d(alphadot_pts);
//    gp_log.send1d(omega_pts);
//    gp_log.send1d(phidot_pts);
//    gp_log << "plot '-' with lines title 'x', '-' with lines title 'y', '-' with lines title 'phi'\n";
//    gp_log.send1d(x_pts);
//    gp_log.send1d(y_pts);
//    gp_log.send1d(phi_pts);
    gp_log << "plot '-' with lines title 'x_dot', '-' with lines title 'y_dot', '-' with lines title 'phi_dot'\n";
    gp_log.send1d(xdot_pts);
    gp_log.send1d(ydot_pts);
    gp_log.send1d(phidot_pts);
    gp_log << "unset multiplot\n";
    
    return 0;
}
