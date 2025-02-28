#ifndef __MILLINGTASK_HPP
#define __MILLINGTASK_HPP

#include "Spline2DCurve.hpp"
#include "utility_med.h"
#include "PathFollowTask.hpp"
#include "MillingTask.hpp"
#include "Controllers.hpp"




class MillingTask:public PathFollowTask
{
    //static PathFollowTask* ptrPFT;
    public:
        MillingTask(char* name,int _N_spline);
         ~MillingTask();
        int DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta);
        int DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1> fe,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta);
        int findCurrentCuttingPoint(double theta, Eigen::Matrix<double,6,1>& pose_task2d_a);
        // int FrenetSerretFrame(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta);
        // int findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose);

    private:
        // int DimReductionTargetFrame(Eigen::Matrix<double,6,1> pose_task_tcp,Eigen::Matrix<double,3,1>& pose_task2d_tcp);
        // virtual int LinY2X2D(Eigen::Matrix<double,3,1> y,Eigen::Matrix<double,3,1>& x);
        // virtual int FromThetaToAgent(double theta, Eigen::Matrix<double,3,1>& y);
        int AdmittanceMatrix(Eigen::Matrix<double,6,1> Base_TCP, 
                            Eigen::Matrix<double,6,1> Base_Task,
                            Eigen::Matrix<double,6,1> twist, 
                            Eigen::Matrix<double,6,1>& delta);
        int Controller(Eigen::Matrix<double,4,4> Task_TCP_T, Eigen::Matrix<double,3,1> pose_fsframe_tcp_l,  
                                Eigen::Matrix<double,6,1> tool_tool_vel_human_guide, Eigen::Matrix<double,6,1>& u);


        int Controller(Eigen::Matrix<double,4,4> Task_TCP_T,
                            Eigen::Matrix<double,3,1> pose_fsframe_tool,  
                            Eigen::Matrix<double,6,1> tool_tool_vel_human_guide, 
                            Eigen::Matrix<double,6,1> tool_tool_vel_environmentRes, 
                            Eigen::Matrix<double,6,1>& v_syn);

        int Controller(Eigen::Matrix<double,4,4> Task_TCP_T,
                        Eigen::Matrix<double,3,1> pose_task2d_fsframe,
                        Eigen::Matrix<double,3,1> pose_task2d_tool,
                            Eigen::Matrix<double,3,1> pose_fsframe_tool,  
                            Eigen::Matrix<double,6,1> tool_tool_vel_human_guide, 
                            Eigen::Matrix<double,6,1> tool_tool_vel_environmentRes, 
                            Eigen::Matrix<double,6,1>& v_syn);
        // virtual int DimReductionTargetFrame(Eigen::Matrix<double,4,4> Task_TCP_T,Eigen::Matrix<double,3,1>& pose_task2d_tcp);

        int FrenetSerretFrame_Plain(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta);


        ControllersSISO* forceController;
        ControllersSISO* MotionControllerX;
        ControllersSISO* MotionControllerY;
        // Eigen::Matrix<double,3,6> DimReductionMatrix;
        // ceres::CostFunction* cost_function;
        // //QuadraticCostFunction* cost_function;
        // ceres::Solver::Options options;
        // ceres::Solver::Summary summary;
        // ceres::Problem problem;

        // Eigen::Matrix<double,2,2> Kp; 
        // Eigen::Matrix<double,2,2> Kd;

        // Eigen::Matrix<double,2,1> p_t_1;
        // Eigen::Matrix<double,2,1> p_t_2; 

        // Eigen::Matrix<double,3,3> admittance_human_guide;
        

        // double _b;
        // double speed_const;
        double vl_cmd_1;

};


#endif