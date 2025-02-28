#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include "math.h"
#include "TD2Order.hpp"
#include <cmath>
#include "utility_med.h"


class ControllersSISO
{
    public:
    ControllersSISO();
    ~ControllersSISO();
    virtual double Update(double input);
    virtual double Update(double r, double y);
};


class RobustForceControl: public ControllersSISO
{
    public:
    RobustForceControl();
    ~RobustForceControl();
    double Update(double input);
    double Update(double r, double y);

    private:
    int parasetZero();
    double a1;
    double a2;
    double a3;
    double a4;
    

    double b0;
    double b1;
    double b2;
    double b3;
    double b4;

    double o_1;
    double o_2;
    double o_3;
    double o_4;

    double x_0;
    double x_1;
    double x_2;
    double x_3;
    double x_4;
    double r_last;
    int status;
    
};


class ARDCMotionControl: public ControllersSISO
{
    public:
        ARDCMotionControl();
        ~ARDCMotionControl();
        double Update(double r, double y); //r : 待跟踪信号， y： 反馈信号

    private:
        double sign(double x);
        double fal(double e, double delta, double alpha);
        int ESO(double yk,double uk_1, double& z1_k, double& z2_k, double& z3_k);
        double NonLinearPD(double e1,double e2);

        TD2Order* tdfilter;

    // parameters for non-linear PD controller
        double alpha1;
        double alpha2;
        double delta;
        double beta1; //P
        double beta2; //D

        // parameters for ESO Observer
        double O_beta1;
        double O_beta2;
        double O_beta3;

        double O_b;
        double O_h;
        double O_alpha1;
        double O_alpha2;
        double O_delta;

        double z1_k_1;
        double z2_k_1;
        double z3_k_1;

        // update memories
        double u_last;
};





#endif