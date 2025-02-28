#ifndef __CAMERACOMMSERVER_HPP
#define __CAMERACOMMSERVER_HPP
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
#include  <sys/signal.h>
#include  <sys/stat.h>
#include  <termios.h>
#include  <errno.h>
#include  <limits.h>
#include  <time.h>
#include  <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include "utility_med.h"

#define SERVER_PORT 8888
#define BUF_SIZE 1500
#define EPOLL_SIZE 2


// enum _Status
// {
//     ZRF=1, BRK, IPD, POS
// };

typedef struct _Diana_Sub_Message
{
    //手眼注册信息 位置
    int flag;
    double hand_eye_registation_pos[3];
    //手眼注册信息 姿态（四元数）
    double hand_eye_registation_ori[4];

    //患者注册信息 位置
    double object_registation_pos[3];
    //患者注册信息 姿态（四元数）
    double object_registation_ori[4];

    // //定义机械臂当前状态 ZRF：灵力拖动，BRK：抱闸，IPD：阻抗控制，POS：位置控制
    // _Status robot_status;
    
}Diana_Sub_Message;

typedef struct _Diana_Pub_Message
{
    //
    int flag;
    double marker_pos[3];
    double marker_ori[4];
    double joint[7];
    
}Diana_Pub_Message;


class CameraComm_Server
{
    static void addfd(int epollfd, int fd, bool enable_et);
    public:
        CameraComm_Server(char* upper_computer);
        ~CameraComm_Server();

        int getRegistInfoHandEye(Eigen::Matrix<double, 7, 1>& regist_info_handeye);
        int getRegistObject(Eigen::Matrix<double, 7, 1>& regist_info_object);
        int setRegistInfoHandEye(const Eigen::Matrix<double, 7, 1> regist_info_handeye);
        int setRegistObject(const Eigen::Matrix<double, 7, 1> regist_info_object);
        int getMarkerPose(Eigen::Matrix<double, 7, 1>& pose);
        int getJoint(Eigen::Matrix<double, 6, 1>& joint);



        //1. 上下位机通信 TCP方式
        struct sockaddr_in serverAddr;
        int listener;
        int epfd;
        int client_num;

    private:
        void Close();
        void CamWorkThread();
        int SendMsgToUpper(int clientfd);

        std::mutex lock_cam;
        std::thread thread_cam;

        Diana_Sub_Message sub_message;

        Diana_Pub_Message pub_message;


        char* _upper_computer;

};

#endif