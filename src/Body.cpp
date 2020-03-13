#include <Eigen/Dense>
#include "Body.h"

#include <iostream>

Body::Body()
{
    x << 0.0f,0.0f,0.0f;
    x_dot << 0.0f,0.0f,0.0f; 
    x_ddot << 0.0f,0.0f,0.0f;
    //Declare constants
    T_VEL << 0,1,0,-1,0,0,0,0,0;
}

Body::Body(float xi=0.0f, float yi=0.0f, float phii=0.0f, float x_doti=0.0f, float y_doti=0.0f, 
float phi_doti=0.0f, float x_ddoti=0.0f, float y_ddoti=0.0f, float phi_ddoti=0.0f)
{
    x << xi , yi , phii;
    x_dot << x_doti, y_doti, phi_doti; 
    x_ddot << x_ddoti, y_ddoti, phi_ddoti;
    //Declare constants
    T_VEL << 0,1,0,-1,0,0,0,0,0;
};

void Body::display() 
{
    printf("x:%.3f, y:%.3f, phi:%.3f, vx:%.3f, vy:%.3f, vphi:%.3f, ax:%.3f, ay:%.3f, aphi:%.3f\n", 
    x(0), x(1), x(2), x_dot(0), x_dot(1), x_dot(2), x_ddot(0), x_ddot(1), x_ddot(2));
}

void Body::set_x(float xi = 0, float yi = 0, float phii = 0)
{
    x << xi, yi, phii;
}

void Body::set_x_dot(float x_doti = 0, float y_doti = 0, float phi_doti = 0)
{
    x_dot << x_doti, y_doti, phi_doti;
}

void Body::set_x_ddot(float x_ddoti = 0, float y_ddoti = 0, float phi_ddoti = 0)
{
    x_ddot << x_ddoti, y_ddoti, phi_ddoti;
}

Eigen::Vector3f Body::get_x()
{
    return x;
}

Eigen::Vector3f Body::get_x_dot()
{
    return x_dot;
}

Eigen::Vector3f Body::get_x_ddot()
{
    return x;
}

void Body::advance(float dt)
{
    q_r = Eigen::AngleAxis<float>(x[2], Eigen::Vector3f(0,0,1));
    x_dot = x_dot + (x_ddot + x_dot[2] * T_VEL * x_dot) * dt;
    x = x + q_r * x_dot * dt;
}
