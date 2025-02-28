#ifndef __ADMITTANCE_CONTROL_HPP
#define __ADMITTANCE_CONTROL_HPP

#include "Spline2DCurve.hpp"
#include "utility_med.h"
#include "math.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <sys/time.h>




class PathFollowTask:public Spline2DCurve
{
    //static PathFollowTask* ptrPFT;
    public:
        PathFollowTask(char* name,int _N_spline);
        virtual ~PathFollowTask();
        virtual int DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta);
        virtual int DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1> fe,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta);
        virtual int FrenetSerretFrame(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta);
        virtual int findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose);

    protected:
        virtual int DimReductionTargetFrame(Eigen::Matrix<double,6,1> pose_task_tcp,Eigen::Matrix<double,3,1>& pose_task2d_tcp);
        virtual int LinY2X2D(Eigen::Matrix<double,3,1> y,Eigen::Matrix<double,3,1>& x);
        virtual int FromThetaToAgent(double theta, Eigen::Matrix<double,3,1>& y);
        virtual int AdmittanceMatrix(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,Eigen::Matrix<double,6,1> twist, Eigen::Matrix<double,3,1>& vel);
        virtual int Controller(Eigen::Matrix<double,3,1> pose_fsframe_tcp_l,  
                                Eigen::Matrix<double,3,1> cmd, Eigen::Matrix<double,3,1>& u);


        virtual int DimReductionTargetFrame(Eigen::Matrix<double,4,4> Task_TCP_T,Eigen::Matrix<double,3,1>& pose_task2d_tcp);

        virtual int FrenetSerretFrame_Plain(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta);

        Eigen::Matrix<double,3,6> DimReductionMatrix;
        ceres::CostFunction* cost_function;
        //QuadraticCostFunction* cost_function;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        ceres::Problem problem;

        Eigen::Matrix<double,2,2> Kp; 
        Eigen::Matrix<double,2,2> Kd;

        Eigen::Matrix<double,2,1> p_t_1;
        Eigen::Matrix<double,2,1> p_t_2; 

        Eigen::Matrix<double,3,3> admittance_human_guide;
        

        double _b;
        double speed_const;

};
/*
    构建类，用于优化
*/
class QuadraticCostFunction : public ceres::SizedCostFunction<2, 1>
{
    public:
        QuadraticCostFunction(PathFollowTask* ptrPF,Eigen::Matrix<double,2,1> pt);
        virtual ~QuadraticCostFunction();
        virtual bool Evaluate(double const* const* parameters,double* residuals,double** jacobians) const;
        //int setObject(PathFollowTask* ptrPF);
        int setCurrentPose(Eigen::Matrix<double,2,1> pt);

    private:
        Eigen::Matrix<double,2,1> _pt;
        PathFollowTask* _ptrPF;
        //Eigen::Matrix<double,2,1> _p;
        //Eigen::Matrix<double,2,1> _pdt;
};

#endif