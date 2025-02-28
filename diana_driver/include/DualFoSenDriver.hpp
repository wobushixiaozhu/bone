#ifndef __DUALFOSEN_HPP
#define __DUALFOSEN_HPP

#include  <stdio.h>
#include  <stdlib.h>
#include  <unistd.h>
#include  <sys/types.h>
#include  <sys/signal.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <termios.h>
#include  <errno.h>
#include  <limits.h>
#include  <string.h>
#include  <time.h>
#include  <deque>

#include <iostream>
#include <memory>

#include <thread>
#include <mutex>
#include <unistd.h>
#include <functional>
#include <Eigen/Dense>
#include "utility_med.h"

typedef bool (*GETWrench)(Eigen::Matrix<double, 6, 1>& wrench);

class DualFoSenDriver
{
    public:
        DualFoSenDriver(char* serialName_ee,char* serialName_hh);
        ~DualFoSenDriver();
        //void startFs();
        //std::shared_ptr<float>  getWrench();
        bool getWrenchhh(Eigen::Matrix<double, 6, 1>& wrench);
        bool getWrenchee(Eigen::Matrix<double, 6, 1>& wrench);
        bool setZeroPoint();


    private:
        int FoSenWorkThread(char* device,float* wwrench,std::mutex& Fo_mutex);
        void detachFs();

        //线程和线程锁
        std::mutex lock_ee;
        std::mutex lock_hh;
        std::thread thread_ee;
        std::thread thread_hh;


        //原始数据
        float wrench_ee[6];
        float wrench_hh[6];

        float wrench_ee_last[6];
        float wrench_hh_last[6];


        //链接地址
        char *device_ee;
        char *device_hh;
        float fsh_bias_ee[6];
        float fsh_bias_hh[6];
        
};

#endif
