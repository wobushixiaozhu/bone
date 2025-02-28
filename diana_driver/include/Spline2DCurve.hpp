#ifndef __SPLINECURVE_HPP
#define __SPLINECURVE_HPP

#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <regex>
#include "utility_med.h"
#include "qpOASES.hpp"




class Spline2DCurve
{
    static void importPathFromFile(char* name,Eigen::MatrixXd& T);
    public:
        Spline2DCurve(char* name,int _N_spline);
        int sigma(const double theta, Eigen::Matrix<double,2,1>& y);
        int sigmaDot(const double theta, Eigen::Matrix<double,2,1>& yDot);
        int sigmaDDot(const double theta, Eigen::Matrix<double,2,1>& yDDot);
        double J(const double theta, const Eigen::Matrix<double,2,1> p);
        double JDot2(const double theta, const Eigen::Matrix<double,2,1> p);
        double JDDot2(const double theta, const Eigen::Matrix<double,2,1> p);
        Eigen::MatrixXd y2D_Task;
        Eigen::Matrix<double,4,4> baseFrame;
        Eigen::MatrixXd xOpt_eigen;
        Eigen::MatrixXd yOpt_eigen;


    private:
        Eigen::Matrix<double,4,4> DimReductionPCA(Eigen::MatrixXd OriMatrix);
        /*flag = 0 代表x方向上的优化
        flag =1 代表y方向上的优化
        */
        void form_H_g_ulbA(Eigen::MatrixXd& A, Eigen::MatrixXd& lb,Eigen::MatrixXd& ub,Eigen::MatrixXd& Qx,bool flag,double Bdbox);
        int CalculateOptParams(bool flag, double Bdbox,Eigen::MatrixXd& xopt);
        void initTimeSegParams();


        int N;
        int N_spline;
        Eigen::MatrixXd tt_vector;
        // double BdboxX;
        // double BdboxY;
        Eigen::MatrixXd y3D_Task;

        Eigen::MatrixXd time_params;
        double TimeSeg;
        Eigen::MatrixXd Nlist_index;




};

#endif
