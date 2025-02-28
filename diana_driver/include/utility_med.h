#ifndef __UTILITY_H
#define __UTILITY_H

#include <thread>
#include <pthread.h>
#include <iostream>
#include <cstring>
#include <Eigen/Dense>
#include <string>
#include <Eigen/Geometry> 
#include <cmath>
#include <fstream>
#include <regex>
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PointStamped.h>


class threadRT : public std::thread
{
  public:
    threadRT(){};
    static void setScheduling(std::thread &th, int policy, int priority) {
        sched_param sch_params;
        sch_params.sched_priority = priority;
        if(pthread_setschedparam(th.native_handle(), policy, &sch_params)) {
            std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        }
    }
  private:
    sched_param sch_params;
};

// Eigen::Matrix<double, 3, 3> Skew_c(const Eigen::Matrix<double, 3, 1> omega);
int Skew_c(const Eigen::Matrix<double, 3, 1> omega,Eigen::Matrix<double, 3, 3>& Omega);
int Skew_inv(const Eigen::Matrix<double, 3, 3> Omega, Eigen::Matrix<double, 3, 1>& omega);
std::string trim(std::string str);
bool isRotationMatirx(Eigen::Matrix3d R);
bool isRotationMatirx(Eigen::Matrix2d R);

void importPointFromFile(std::string name,Eigen::MatrixXd& T);
void importPointFromFile(char* name,Eigen::MatrixXd& T);

int rotationMatrixToEulerAngles(Eigen::Matrix3d R, Eigen::Matrix<double,3,1>& theta);
int rotationMatrixToEulerAngles(Eigen::Matrix2d R, double& theta);
int TwistTransMatrix(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,6>& A_B_tTransM);
int TwistTransMatrix(Eigen::Matrix<double,3,3> A_B_T,Eigen::Matrix<double,3,3>& A_B_tTransM);
int WrenchTransMatrix(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,6>& A_B_tTransM);

int PoseToTransform(Eigen::Matrix<double,6,1> A_B,Eigen::Matrix<double,4,4>& A_B_T);
int PoseToTransform(Eigen::Matrix<double,3,1> A_B,Eigen::Matrix<double,3,3>& A_B_T);
int TransformToPose(Eigen::Matrix<double,3,3> A_B_T,Eigen::Matrix<double,3,1>& A_B);
int PoseTransform2D(Eigen::Matrix<double,3,1> pose_A_B,Eigen::Matrix<double,3,1> pose_A_C,Eigen::Matrix<double,3,1>& pose_B_C);
int TransformToPose(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,1>& A_B);
int rotationMatrixToAxisAngle(Eigen::Matrix<double,3,3> R,Eigen::Matrix<double,3,1>& r);
//int TransformToPose(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,1>& A_B);
int EulerRPY2Quat(Eigen::Matrix<double,3,1> eulerAngle,Eigen::Matrix<double,4,1>& quaternion);
int EulerPoseRPY2QuatPose(Eigen::Matrix<double,6,1> pose_eul,Eigen::Matrix<double,7,1>& pose_q);
int Quat2EulerRPY(Eigen::Matrix<double,4,1>,Eigen::Matrix<double,3,1>& eulerAngle);
int QuatPose2EulerPoseRPY(Eigen::Matrix<double,7,1> pose_q,Eigen::Matrix<double,6,1>& pose_eul);

int PoseTransform3D(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,Eigen::Matrix<double,6,1>& Task_TCP);
int PoseTransform3D(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,Eigen::Matrix<double,4,4>& Task_TCP_T);

int PoseRPY2Transform(Eigen::Matrix<double,6,1> A_B, Eigen::Matrix<double,4,4>& A_B_T);
int rotationMatrixToQuat(Eigen::Matrix<double,3,3> R,Eigen::Matrix<double,4,1>& q);

int AxisAnglePose2QuatPose(Eigen::Matrix<double,6,1> pose_a,Eigen::Matrix<double,7,1>& pose_q);
int AxisAngle2Quat(Eigen::Matrix<double,3,1> AxisAngle,Eigen::Matrix<double,4,1>& quaternion);

int QuatPose2AxisAnglePose(Eigen::Matrix<double,7,1> pose_q,Eigen::Matrix<double,6,1>& pose_a);
int Quat2AxisAngle(Eigen::Matrix<double,4,1> quaternion,Eigen::Matrix<double,3,1>& AxisAngle);
void saveData(std::string fileName, Eigen::MatrixXd  matrix);

#endif