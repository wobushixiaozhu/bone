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


class FoSenDriverBIT
{
    public:
        FoSenDriverBIT();
        FoSenDriverBIT(char* serialName);
        int start(char* serialName);
        std::shared_ptr<float>  getWrench();

    private:
        int FoSenWorkThread();
        float Wrench[6];
        //用于链接力传感器
        struct termios OnesensorTermios_old;
        struct termios OnesensorTermios;

        //链接地址
        char *device;

        //缓存中的数据buffer
        std::deque<unsigned char> receiveddata;

        //串口的句柄
        int Serialhandle;

        //中间数据
        unsigned char    data[28];
        unsigned char t[4];
        union xxx{char m[4]; float n;}z;

        //线程和线程锁
        std::mutex Fo_mutex;
        
};
