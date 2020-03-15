#pragma once

class Body
{
private:
    Eigen::Vector3f x;
    Eigen::Vector3f x_dot;
    Eigen::Vector3f x_ddot;
    Eigen::Quaternion<float> q_r;
    Eigen::Matrix3f T_VEL;
    Eigen::Vector3f e_z;
    
public:
    Body();
    Body(float xi, float yi, float phii, float x_doti, float y_doti, 
    float phi_doti, float x_ddoti, float y_ddoti, float phi_ddoti);
    void display();
    void set_x(float xi, float yi, float phii);
    void set_x_dot(float x_doti, float y_doti, float phi_doti);
    void set_x_ddot(float x_ddoti, float y_ddoti, float phi_ddoti);
    Eigen::Vector3f get_x();
    Eigen::Vector3f get_x_dot();
    Eigen::Vector3f get_x_ddot();
    void advance(float dt);
};