#include "PlotSim.h"

PlotSim::PlotSim()
{
    gp << "set size ratio -1\n";
    heads.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    v_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    a_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
    orig_pts.push_back(boost::make_tuple(0.0,0.0,0.0,0.0));
}

void PlotSim::draw_update(DataLogger &logger)
{
    state = logger.get_latest_state();
    
    xy_pts.push_back(std::make_pair(state[1], state[2]));
    heads.pop_back();
    heads.push_back(boost::make_tuple(state[1],state[2], cos(state[3]), sin(state[3])));
    orig_pts.pop_back();
    orig_pts.push_back(boost::make_tuple(0.0f, 0.0f, state[1], state[2]));
    v_pts.pop_back();
    v_pts.push_back(boost::make_tuple(state[1], state[2], state[4] * cos(state[3]) - state[5] * sin(state[3]), state[4] * sin(state[3]) + state[5] * cos(state[3])));
    a_pts.pop_back();
    a_pts.push_back(boost::make_tuple(state[1], state[2], state[7] * cos(state[3]) - state[8] * sin(state[3]), state[7] * sin(state[3]) + state[8] * cos(state[3])));
    
    if (state[0] > freeze_time)
    {
        heads.push_back(boost::make_tuple(state[1],state[2], cos(state[3]), sin(state[3])));
        freeze_time += 1.0f;
    }
    
    gp << "plot '-' binary" << gp.binFmt1d(xy_pts, "record") << 
    "with lines title 'path', '-' with vectors title 'heading', '-' with vectors title 'I r IB', '-' with vectors title 'I v IB', '-' with vectors title 'I a IB'\n";
    gp.sendBinary1d(xy_pts);
    gp.send1d(heads);
    gp.send1d(orig_pts);
    gp.send1d(v_pts);
    gp.send1d(a_pts);

    gp.flush();
}

void PlotSim::draw_result(DataLogger &logger)
{
    states = logger.get_states();
    
    for (int col = 0; col < states.cols(); col++)
    {
//        x_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[1]));
//        y_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[2]));
        phi_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[3]));
        alpha_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[10]));
        alphadot_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[11]));
        beta_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[12]));
        betadot_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[13]));
        ddphi_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[9]));
        xdot_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[4]));
        ydot_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[5]));
        phidot_pts.push_back(std::make_pair(states.col(col)[0], states.col(col)[6]));
    }
    
    gp_log << "set multiplot layout 1,3\n";
    gp_log << "plot '-' with lines title 'beta', '-' with lines title 'beta_{dot}'\n";
    gp_log.send1d(beta_pts);
    gp_log.send1d(betadot_pts);
    
    gp_log << "plot '-' with lines title 'alpha', '-' with lines title 'alpha_{dot}'\n";
    gp_log.send1d(alpha_pts);
    gp_log.send1d(alphadot_pts);

    gp_log << "plot '-' with lines title 'x_dot', '-' with lines title 'y_dot', '-' with lines title 'phi_dot'\n";
    gp_log.send1d(xdot_pts);
    gp_log.send1d(ydot_pts);
    gp_log.send1d(phidot_pts);
    gp_log << "unset multiplot\n";
}