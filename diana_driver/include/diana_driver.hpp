#ifndef __DIANA_DRIVER_HPP
#define __DIANA_DRIVER_HPP

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/epoll.h>

#include <functional>

#include <iostream>

#include <fcntl.h>

#include "DianaAPIDef.h"
#include "DianaAPI.h"
#include "utility_med.h"
#include <time.h>
#include <mutex>
#include <sys/time.h>
#include <Eigen/Dense>


// #include "DualFoSenDriver.hpp"


// typedef struct _Diana_Pub_Message
// {
//     //1.TCP frame
//     double tcp_frame_pos[3];
//     double tcp_frame_ori[4];

//     //2. Marker Frame
//     double Marker_frame_pos[3];
//     double Marker_frame_ori[4];

//     //3.Robot Joint
//     double robot_joints[4];

//     _Status robot_status;

    
// }Diana_Pub_Message;

typedef enum _ControlMode {
    T_MODE_NONE = 0,
    T_MODE_RT,
    T_MODE_SCRIPT,
} mode_c;



class DianaDriver
{
    static void _StateFeedback(StrRobotStateInfo *pinfo);//回调函数1
    static void _errorControl(int e);//回调函数2
    static DianaDriver* CurMy;
    public:
        DianaDriver(char* ip_name);
        ~DianaDriver();

        //机械臂工作主函数


        int DianaReleaseBrake();
        int DianaholdBrake();
        void DianaStop();

        int setEquilibriumPoint(const Eigen::Matrix<double, 6, 1> pose);
        int getCurrentTCPPose(Eigen::Matrix<double, 6, 1>& pose);
        int getPoseBsToTCP(Eigen::Matrix<double, 4, 4>& T);
        int getJointState(Eigen::Matrix<double,7,1>& joint_state);
        int setVel(const Eigen::Matrix<double, 6, 1> vel);
        int DIANAchangeControlMode(mode_e m);
        void SwitchRTMode(mode_c mode);
        int DIANAmoveJToPose(Eigen::Matrix<double,6,1> target);
        mode_c getControlMode();

        //控制柜给上位机发送的消息内存
        srv_net_st* pinfo;
        





    private:
        void StateFeedback(StrRobotStateInfo *pinfo);
        void errorControl(int e);
        void startDiana();
        void Close();
        void setCurMy();
        void wait_move();

        // Diana_Pub_Message pub_message;
        // Diana_Sub_Message sub_message;
        StrRobotStateInfo robot_state;
        mode_c mode_robot;
        std::mutex lock_robot;
        std::mutex lock_controller;
        std::mutex lock_robot_mode;
        double EquilibriumPoint[6];
        double speed[6];
        char* _ip_name;


};

#endif
