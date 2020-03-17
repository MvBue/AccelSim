#include "PlotSim.h"


PlotSim::PlotSim()
{
    gp << "set size ratio -1\n";
    heads.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    v_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    a_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    orig_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
}

void PlotSim::append_states(int &i, float &sim_time, Body &B, float &alpha, float &alpha_dot, float &beta,
Eigen::Vector3f &beta_dot, Eigen::Vector3f &I_v_IB, Eigen::Vector3f &I_a_IB)
{
    r_i = B.get_r();
    phi_i = B.get_phi();
    v_i = B.get_v();
    omega_i = B.get_omega();
    a_i = B.get_a();
    psi_i = B.get_psi();
    
    xy_pts.push_back(std::make_pair(r_i[0], r_i[1]));
//    x_pts.push_back(std::make_pair(sim_time, x_i[0]));
//    y_pts.push_back(std::make_pair(sim_time, x_i[1]));
    phi_pts.push_back(std::make_pair(sim_time, r_i[2]));
    alpha_pts.push_back(std::make_pair(sim_time, alpha));
    alphadot_pts.push_back(std::make_pair(sim_time, alpha_dot));
    beta_pts.push_back(std::make_pair(sim_time, beta));
    betadot_pts.push_back(std::make_pair(sim_time, beta_dot[2]));
    ddphi_pts.push_back(std::make_pair(sim_time, psi_i[2]));
    xdot_pts.push_back(std::make_pair(sim_time, v_i[0]));
    ydot_pts.push_back(std::make_pair(sim_time, v_i[1]));
    phidot_pts.push_back(std::make_pair(sim_time, omega_i[2]));
    
    heads.pop_back();
    heads.push_back(boost::make_tuple(r_i[0],r_i[1], cos(phi_i[2]), sin(phi_i[2])));
    orig_pts.pop_back();
    orig_pts.push_back(boost::make_tuple(0.0f, 0.0f, r_i[0], r_i[1]));
    v_pts.pop_back();
    v_pts.push_back(boost::make_tuple(r_i[0],r_i[1], I_v_IB[0], I_v_IB[1]));
    a_pts.pop_back();
    a_pts.push_back(boost::make_tuple(r_i[0],r_i[1], I_a_IB[0], I_a_IB[1]));
    if (i%50==0)
    {
        heads.push_back(boost::make_tuple(r_i[0],r_i[1], cos(phi_i[2]), sin(phi_i[2])));
    }
}

void PlotSim::draw_update()
{
    gp << "plot '-' binary" << gp.binFmt1d(xy_pts, "record") << 
    "with lines title 'path', '-' with vectors title 'heading', '-' with vectors title 'I r IB', '-' with vectors title 'I v IB', '-' with vectors title 'I a IB'\n";
    gp.sendBinary1d(xy_pts);
    gp.send1d(heads);
    gp.send1d(orig_pts);
    gp.send1d(v_pts);
    gp.send1d(a_pts);

    gp.flush();
}

void PlotSim::draw_result()
{
    gp_log << "set multiplot layout 1,2\n";
    gp_log << "plot '-' with lines title 'beta', '-' with lines title 'beta_{dot}', '-' with lines title 'ddphi'\n";
    gp_log.send1d(beta_pts);
    gp_log.send1d(betadot_pts);
    gp_log.send1d(ddphi_pts);

    gp_log << "plot '-' with lines title 'x_dot', '-' with lines title 'y_dot', '-' with lines title 'phi_dot'\n";
    gp_log.send1d(xdot_pts);
    gp_log.send1d(ydot_pts);
    gp_log.send1d(phidot_pts);
    gp_log << "unset multiplot\n";
}