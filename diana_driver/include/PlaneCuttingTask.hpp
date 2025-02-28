#ifndef __ADMITTANCE_CONTROLP_HPP
#define __ADMITTANCE_CONTROLP_HPP

#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <regex>

#include "utility_med.h"
#include "math.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <sys/time.h>




class PlaneCuttingTask
{
    //static PathFollowTask* ptrPFT;
    static void importPathFromFile(char* name,Eigen::MatrixXd& T);
    public:
        PlaneCuttingTask(char* name);
        virtual ~PlaneCuttingTask();
        virtual int findbaseFrame(char* name, Eigen::Matrix<double,4,4>& baseFrame);
        virtual int DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta);
        virtual int findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose);
        Eigen::Matrix<double,4,4> baseFrame;


};

#endif