#ifndef __TD2ORDER_HPP
#define __TD2ORDER_HPP

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>

int sign(int x);
double fhan(double x1, double x2, double u,
                double r,double h);

class TD2Order
{
    public:
        TD2Order(double para_Ts, double para_r, double para_h);
        int update(double u, double& x1, double& x2);

    private:
        double x_1;
        double x_2;
        double Ts;
        double r;
        double h;

};





#endif