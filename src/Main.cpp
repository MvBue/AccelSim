#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <boost/tuple/tuple.hpp>

#include <vector>
#include <utility>
#include "gnuplot-iostream.h"

#include "Body.h"

using namespace Eigen;

float get_ddy(int &i)
{
    float y;
    
//    y = 10.0 * sin(float(i)/100.0f*2.0f*M_PI);
    
    if (i < 300)
    {
        y = i * 10.0f / 300.0f;
    }
    else
    {
        y = 0.0f;
    }
    
    return y;
}

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
	std::vector<boost::tuple<double, double, double, double> > ddx_pts;
	std::vector<boost::tuple<double, double, double, double> > orig_pts;
    heads.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    ddx_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    orig_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    
    float alpha;
    float alpha_dot;
    float beta;
    
//  Set variables
    float ddphi = 0.0f;
//    float k_p = 5.0f; //use with alpha
//    float k_d = 3.0f; //use with alpha
    float k_p = 1.0f;
    float k_d = 1.0f;
    
//    float ddy = 10.0f;
//    for (int i = 0; i < 501; i++)
//    {
//        ddy[i] = i * 10.0f / 500.0f;
//    }

    
    Body B;
    B.set_x_dot(0,0,0);
    int temp = 0;
    B.set_x_ddot(0,get_ddy(temp),0);
        
//  Start simulation
    int sim_steps = 801;
    float sim_dt = 0.01;
    float sim_time = 0;
    
    Vector3f x_i = B.get_x();
    Vector3f x_dot_i = B.get_x_dot();
    Vector3f x_ddot_i = B.get_x_ddot();
    Vector3f e_x(1,0,0);
    Vector3f e_z(0,0,1);
    Vector3f origin_vec; 
    Vector3f v_abs_vec;
    Vector3f heading;
    Vector3f x_ddot_vec;
    Vector3f e_x_T;
    Vector3f beta_dot;
    Eigen::Quaternion<float> q_r;
    Eigen::Quaternion<float> q_90;
    q_r = AngleAxis<float>(x_i[2], e_z);
    q_90 = AngleAxis<float>(M_PI/2.0, e_z);
    Matrix3f R_get_xy;
    R_get_xy << 1,0,0,
                0,1,0,
                0,0,0;
    Matrix3f R_get_z;
    R_get_z << 0,0,0,
                0,0,0,
                0,0,1;

    Gnuplot gp;
    Gnuplot gp_log;
    gp << "set size ratio -1\n";
    
    for (int i = 0; i < sim_steps; i++)
    {
//        std::cout << sim_time << std::endl;
        
        x_i = B.get_x();
        x_dot_i = B.get_x_dot();
        x_ddot_i = B.get_x_ddot();
        origin_vec = -1.0 * R_get_xy * x_i;
        q_r = AngleAxis<float>(x_i[2], e_z);
        x_ddot_vec = q_r * R_get_xy * x_ddot_i;
        v_abs_vec = q_r * R_get_xy * x_dot_i;
        heading = q_r * e_x;
        e_x_T = q_90 * origin_vec * -1.0 / origin_vec.norm();
        
        alpha = atan2f(heading.cross(v_abs_vec)[2],heading.dot(v_abs_vec));
//        alpha = atan2f(x_dot_i[1],x_dot_i[0]);
        
        std::cout << sim_time << "  " << beta << "  " << beta_dot[2] << std::endl;
        std::cout << sim_time << "  " << alpha << "  " << alpha_dot << std::endl;
//        std::cout << "x" << v_abs_vec.norm() << std::endl;
//        std::cout << "v" << (v_abs_vec.dot(e_x_T) * e_x_T).norm() << std::endl;
//        std::cout << sim_time << "  " << aalpha << "  " << v_abs_vec[0] << "  " << v_abs_vec[1] << "  " << x_dot_i[0] << "  " << x_dot_i[1] << std::endl;
        
        if (v_abs_vec.norm()==0)
        {
            alpha_dot = x_dot_i[2];
        }
        else 
        {
            alpha_dot = x_dot_i[2] - get_ddy(i) * cos(alpha) / v_abs_vec.norm();
            // a_z = ddy * cos(alpha); omega = a_z / (sqrtf(pow(x_dot_i[0],2) + pow(x_dot_i[1],2))); 
            //alpha_dot = x_dot_i[2] - omega;
        }
        
        if (origin_vec.norm() == 0 || v_abs_vec.norm() == 0)
        {
            beta = 0;
            beta_dot << 0, 0, 0;
        } else if (x_ddot_vec.norm() == 0)
        {
            beta = atan2f(heading.cross(origin_vec)[2],heading.dot(origin_vec));;
            beta_dot = (-1.0 * origin_vec).cross(v_abs_vec.dot(e_x_T) * e_x_T) / powf(origin_vec.norm(),2) - R_get_z * x_dot_i;
        } else
        {
            beta = atan2f(x_ddot_vec.cross(origin_vec)[2],x_ddot_vec.dot(origin_vec));
            beta_dot = (-1.0 * origin_vec).cross(v_abs_vec.dot(e_x_T) * e_x_T) / powf(origin_vec.norm(),2) - R_get_z * x_dot_i;
        }
        
        
//        ddphi = k_p * alpha - k_d * alpha_dot;
        ddphi = k_p * beta + k_d * beta_dot[2];
//        float dot = x_i[0]*x_i[1] + 
        
        B.set_x_ddot(0,get_ddy(i),ddphi);
        
        xy_pts.push_back(std::make_pair(x_i[0], x_i[1]));
//        x_pts.push_back(std::make_pair(sim_time, x_i[0]));
//        y_pts.push_back(std::make_pair(sim_time, x_i[1]));
        phi_pts.push_back(std::make_pair(sim_time, x_i[2]));
        alpha_pts.push_back(std::make_pair(sim_time, beta));
        alphadot_pts.push_back(std::make_pair(sim_time, beta_dot[2]));
        ddphi_pts.push_back(std::make_pair(sim_time, ddphi));
        xdot_pts.push_back(std::make_pair(sim_time, x_dot_i[0]));
        ydot_pts.push_back(std::make_pair(sim_time, x_dot_i[1]));
        phidot_pts.push_back(std::make_pair(sim_time, x_dot_i[2]));
        
        heads.pop_back();
        heads.push_back(boost::make_tuple(x_i[0],x_i[1], cos(x_i[2]), sin(x_i[2])));
        ddx_pts.pop_back();
        ddx_pts.push_back(boost::make_tuple(x_i[0],x_i[1], origin_vec[0], origin_vec[1]));
        orig_pts.pop_back();
        orig_pts.push_back(boost::make_tuple(x_i[0],x_i[1], (v_abs_vec.dot(e_x_T) * e_x_T)[0], (v_abs_vec.dot(e_x_T) * e_x_T)[1]));
        if (i%30==0)
        {
            heads.push_back(boost::make_tuple(x_i[0],x_i[1], cos(x_i[2]), sin(x_i[2])));
        }
        
        gp << "plot '-' binary" << gp.binFmt1d(xy_pts, "record") << 
        "with lines title 'xy', '-' with vectors title 'heading', '-' with vectors title 'v_abs_vec', '-' with vectors title 'proj'\n";
        gp.sendBinary1d(xy_pts);
        gp.send1d(heads);
        gp.send1d(ddx_pts);
        gp.send1d(orig_pts);

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
