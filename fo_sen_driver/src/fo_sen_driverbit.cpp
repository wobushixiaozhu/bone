#include  "fo_sen_driverbit.h"

FoSenDriverBIT::FoSenDriverBIT(){
    
}

FoSenDriverBIT::FoSenDriverBIT(char* serialName){

    //1. 参数初始化
    device = serialName;
    // 6.开启线程
    std::thread work_thread(std::bind(&FoSenDriverBIT::FoSenWorkThread,this));
    work_thread.detach();

}

int FoSenDriverBIT::start(char* serialName)
{
    //1. 参数初始化
    device = serialName;
    // 6.开启线程
    std::thread work_thread(std::bind(&FoSenDriverBIT::FoSenWorkThread,this));
    work_thread.join();

}

std::shared_ptr<float> FoSenDriverBIT::getWrench()
{
    //设置新的数组
    Fo_mutex.lock();
    std::shared_ptr<float> wrench(new float[6]);

    //赋值
    wrench.get()[0] = Wrench[0];
    wrench.get()[1] = Wrench[1];
    wrench.get()[2]  = Wrench[2];
    wrench.get()[3] = Wrench[3];
    wrench.get()[4]  = Wrench[4];
    wrench.get()[5]  = Wrench[5];
    Fo_mutex.unlock();

    return wrench;
}

int FoSenDriverBIT::FoSenWorkThread()
{

    //2. 打开串口
    Serialhandle = open(device,O_RDWR | O_NOCTTY |O_NONBLOCK);
    if(Serialhandle<0)
    {
        printf("Open serial port /dev/ttyUSB0 failed!\n");
        return -1;
    }

    //3. c处理终端格式
    tcgetattr(Serialhandle , &OnesensorTermios_old);
    bzero( &OnesensorTermios, sizeof(OnesensorTermios));
    cfmakeraw(&OnesensorTermios);

    //4. 通信格式
    OnesensorTermios.c_cflag=B460800;
    OnesensorTermios.c_cflag |= CLOCAL | CREAD;
    OnesensorTermios.c_cflag &= ~CSIZE;
    OnesensorTermios.c_cflag |= CS8;
    OnesensorTermios.c_cflag &= ~PARENB;
    OnesensorTermios.c_cflag &= ~CSTOPB;
    tcflush(Serialhandle,TCIFLUSH);
    tcflush(Serialhandle,TCOFLUSH);
    OnesensorTermios.c_cc[VTIME] = 1;
    OnesensorTermios.c_cc[VMIN] = 1;
    tcflush (Serialhandle, TCIFLUSH);
    tcsetattr(Serialhandle,TCSANOW,&OnesensorTermios);

    //5.请求力传感器发送数据
    char writedata[10]= {0};
    writedata[0] = 0x48;
    writedata[1] = 0xAA;
    writedata[2] = 0x0D;
    writedata[3] = 0x0A;
    int len = 0, total_len = 0;
    for (total_len = 0 ; total_len < 4;)
    {
        len = 0;
        len = write(Serialhandle, &writedata[total_len], 4 - total_len);
        printf("WriteData fd = %d ,len =%d,data = %s\n",Serialhandle,len,writedata);
        if (len > 0)
        {
            total_len += len;
        }
        else if(len <= 0)
        {
            len = -1;
            break;
        }
    }
    if(len < 0)
    {
        printf("Write Data Fail!\n");
        return -1;
    }

    //read and deal
    unsigned char readdata[150];
    memset(readdata,0,150);//the length//////////////////////////////
    int max_fd = 0;
    fd_set readset = {0};
    struct timeval tv = {0};
    int i;
    int ReceivedDataLangth;
    while(1)
    {
        len = 0;
        FD_ZERO(&readset);
        FD_SET((unsigned int)Serialhandle, &readset);
        max_fd = Serialhandle +1;
        tv.tv_sec=0;
        tv.tv_usec=0;
        if (select(max_fd, &readset, NULL, NULL,&tv ) < 0)
        {
            printf("ReadData: select error\n");
            return -1;
        }
        int nRet = FD_ISSET(Serialhandle, &readset);
        if (nRet)
        {
            len = read(Serialhandle, readdata, 150);
        }
        if( len > 0 )
        {
            for(i=0;i<len;i++)
                receiveddata.push_back(readdata[i]);
            //for(int i=0;i<m_rec_data_length;i++)
            //printf(" %d ",m_rec_data[i]);
            //printf("\n");
        }

        while(receiveddata.size()>0)
        {
            ReceivedDataLangth = receiveddata.size();  //缓存目前收到数据的长度，以免循环过程中有新数据写入或读出影响操作
            if((ReceivedDataLangth>=28)&&(receiveddata.at(26) == 0x0d)&&(receiveddata.at(27) == 0x0a))
            {
                for(i =0;i<28;i++)
                {
                    data[i] = receiveddata.front();
                    receiveddata.pop_front();
                }
                for(i=0;i<4;i++)
                {
                    z.m[i]=data[i+2];
                }
                Wrench[0]=z.n;
                for(i=0;i<4;i++)
                {
                    z.m[i]=data[i+6];
                }
                Wrench[1]=z.n;
                for(i=0;i<4;i++)
                {
                    z.m[i]=data[i+10];
                }
                Wrench[2]=z.n;
                for(i=0;i<4;i++)
                {
                    z.m[i]=data[i+14];
                }
                Wrench[3]=z.n;
                for(i=0;i<4;i++)
                {
                    z.m[i]=data[i+18];
                }
                Wrench[4]=z.n;
                for(i=0;i<4;i++)
                {
                    z.m[i]=data[i+22];
                }
                Wrench[5]=z.n;
                printf("BIT:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",Wrench[0],Wrench[1],Wrench[2],Wrench[3],Wrench[4],Wrench[5]);
            }
            else if((ReceivedDataLangth>=28)&&(ReceivedDataLangth<120))
            {
                if(receiveddata.at(0) == 0x0a)
                    receiveddata.pop_front();
                else
                {
                    i=0;
                    while ((i<=28)&&(receiveddata.at(0)!=0x0d)&&(receiveddata.at(1)!=0x0a))
                    {
                        receiveddata.pop_front();
                        i++;
                    }
                    if(receiveddata.size()>=2)
                    {
                        receiveddata.pop_front();
                        receiveddata.pop_front();
                    }
                }
            }
            else if(ReceivedDataLangth >= 120)
                receiveddata.clear();
            else if(ReceivedDataLangth < 28)
                break;
        }
    }

}




int main(int argc, char *argv[])
{
    char* device;
    device="/dev/ttyUSB0";
    FoSenDriverBIT fsensor;
    fsensor.start(device);
    std::shared_ptr<float> Wrench;

    while(1)
    {
        usleep(100000);
        Wrench =fsensor.getWrench();
        //printf("BIT:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",Wrench.get()[0],Wrench.get()[1],Wrench.get()[2],Wrench.get()[3],Wrench.get()[4],Wrench.get()[5]);

    }
    return 0;
}


