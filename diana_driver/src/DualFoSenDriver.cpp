#include "DualFoSenDriver.hpp"

/*
双力传感器构造函数：
用于传递变量参数名称
输入： example: “/dev/ttyUSB0,/dev/ttyUSB1”
./diana_driver_node dualFoSen_node '/dev/ttyUSB0' '/dev/ttyUSB1'
*/

DualFoSenDriver::DualFoSenDriver(char* serialName_ee,char* serialName_hh)
{
    device_ee = serialName_ee;
    device_hh = serialName_hh;

    bzero( wrench_ee, sizeof(wrench_ee));
    bzero( wrench_hh, sizeof(wrench_hh));

    bzero( wrench_hh_last, sizeof(wrench_hh_last));
    bzero( wrench_ee_last, sizeof(wrench_ee_last));

    bzero( fsh_bias_ee, sizeof(fsh_bias_ee));
    bzero( fsh_bias_hh, sizeof(fsh_bias_hh));

    //开启线程，用于初始化力传感器
    //     thread_ee = std::thread(std::bind(&DualFoSenDriver::FoSenWorkThread,device_ee,wrench_ee,lock_ee));
    // thread_hh = std::thread(std::bind(&DualFoSenDriver::FoSenWorkThread,device_hh,wrench_hh,lock_hh));  
    
    // changed !!!
    thread_ee = std::thread(std::mem_fn(&DualFoSenDriver::FoSenWorkThread),this,device_ee,wrench_ee,std::ref(lock_ee));
    thread_hh = std::thread(std::mem_fn(&DualFoSenDriver::FoSenWorkThread),this,device_hh,wrench_hh,std::ref(lock_hh));
    threadRT::setScheduling(thread_ee, SCHED_RR, 50);
    threadRT::setScheduling(thread_hh, SCHED_RR, 50);   
}

/*
双力传感器析构函数：
用于释放线程资源
*/
DualFoSenDriver::~DualFoSenDriver()
{
    detachFs();
}

// /*
// 双力传感器开始函数：
// 用于初始化力传感器，
// */
// void DualFoSenDriver::startFs()
// {

//     // int ret = FoSenWorkThread();
//     // printf("[error] %d \n",ret);
//     // 6.开启线程
//     thread_ee = std::thread(std::bind(&FoSenWorkThread,this,device_ee,wrench_ee,lock_ee));
//     thread_hh = std::thread(std::bind(&FoSenWorkThread,this,device_hh,wrench_hh,lock_hh));
//     //work_thread.detach();

// }

/*
释放线程资源
*/
void DualFoSenDriver::detachFs()
{
    thread_ee.detach();
    thread_hh.detach();
}


/*
双六位力传感器设置零点：
输入（六维力传感器eigen格式，函数指针）
fsh_bias:  待修改的eigen变量格式，物理意义为零点。
getWrench：选择getWrenchhh和~ee中的其中一个，用于确定力传感器
*/
bool DualFoSenDriver::setZeroPoint()
{
    lock_hh.lock();
    //1
    fsh_bias_hh[0] = 1.00*wrench_hh[0];
    fsh_bias_hh[1] = 1.00*wrench_hh[1];
    fsh_bias_hh[2] = 1.00*wrench_hh[2];
    fsh_bias_hh[3] = 1.00*wrench_hh[3];
    fsh_bias_hh[4] = 1.00*wrench_hh[4];
    fsh_bias_hh[5] = 1.00*wrench_hh[5];
    lock_hh.unlock();

    lock_ee.lock();
    //2
    fsh_bias_ee[0] = 1.00*wrench_ee[0];
    fsh_bias_ee[1] = 1.00*wrench_ee[1];
    fsh_bias_ee[2] = 1.00*wrench_ee[2];
    fsh_bias_ee[3] = 1.00*wrench_ee[3];
    fsh_bias_ee[4] = 1.00*wrench_ee[4];
    fsh_bias_ee[5] = 1.00*wrench_ee[5];
    lock_ee.unlock();

    return true;
}

// bool DualFoSenDriver::getWrenchhh_bias(Eigen::Matrix<double, 6, 1>& Wrench)
// {
//     //设置新的数组
//     // Eigen::Matrix<double, 6, 1> wrench{};
//     float fs_filter_param = 1.00;
//     //赋值
//     lock_hh.lock();
//     Wrench[0] = fs_filter_param*fsh_bias_hh[0];
//     Wrench[1] = fs_filter_param*fsh_bias_hh[1];
//     Wrench[2] = fs_filter_param*fsh_bias_hh[2];
//     Wrench[3] = fs_filter_param*fsh_bias_hh[3];
//     Wrench[4] = fs_filter_param*fsh_bias_hh[4];
//     Wrench[5] = fs_filter_param*fsh_bias_hh[5];
//     lock_hh.unlock();

//     return true;
// }


// bool DualFoSenDriver::setZeroPoint(Eigen::Matrix<double, 6, 1>& fsh_bias,GETWrench getWrench)
// {
//     Eigen::Matrix<double, 6, 1> fsh;
//     if(!(*getWrench)(fsh))
//     {
//         return false;
//     }
//     //printf("BIT:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",fsh[0],fsh[1],fsh[2],fsh[3],fsh[4],fsh[5]);


//     if(fsh.norm()==0)
//     {
//         printf("%d\n",fsh[0]);
//         return false;
//     }

//     fsh_bias = fsh;
//     return true;
// }



/*
获取hh交互的力传感器信息
*/
bool DualFoSenDriver::getWrenchhh(Eigen::Matrix<double, 6, 1>& Wrench)
{
    //设置新的数组
    // Eigen::Matrix<double, 6, 1> wrench{};
    float fs_filter_param = 1.0;
    bool flag = false;
    //赋值
    lock_hh.lock();
    Wrench[0] = fs_filter_param*wrench_hh[0]+(1-fs_filter_param)*wrench_hh_last[0]-fsh_bias_hh[0];
    Wrench[1] = fs_filter_param*wrench_hh[1]+(1-fs_filter_param)*wrench_hh_last[1]-fsh_bias_hh[1];
    Wrench[2] = fs_filter_param*wrench_hh[2]+(1-fs_filter_param)*wrench_hh_last[2]-fsh_bias_hh[2];
    Wrench[3] = fs_filter_param*wrench_hh[3]+(1-fs_filter_param)*wrench_hh_last[3]-fsh_bias_hh[3];
    Wrench[4] = fs_filter_param*wrench_hh[4]+(1-fs_filter_param)*wrench_hh_last[4]-fsh_bias_hh[4];
    Wrench[5] = fs_filter_param*wrench_hh[5]+(1-fs_filter_param)*wrench_hh_last[5]-fsh_bias_hh[5];
    flag = true;
    lock_hh.unlock();

    wrench_hh_last[0] = wrench_hh[0];
    wrench_hh_last[1] = wrench_hh[1];
    wrench_hh_last[2] = wrench_hh[2];
    wrench_hh_last[3] = wrench_hh[3];
    wrench_hh_last[4] = wrench_hh[4];
    wrench_hh_last[5] = wrench_hh[5];
    // printf("bias:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",fsh_bias_hh[0],fsh_bias_hh[1],fsh_bias_hh[2],fsh_bias_hh[3],fsh_bias_hh[4],fsh_bias_hh[5]);
    // printf("origin:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",wrench_hh[0],wrench_hh[1],wrench_hh[2],wrench_hh[3],wrench_hh[4],wrench_hh[5]);
    // printf("output:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",Wrench[0],Wrench[1],Wrench[2],Wrench[3],Wrench[4],Wrench[5]);

    return flag;
}


/*
获取ee交互的力传感器信息
*/
bool DualFoSenDriver::getWrenchee(Eigen::Matrix<double, 6, 1>& Wrench)
{
    //设置新的数组
    // Eigen::Matrix<double, 6, 1> wrench{};
    float fs_filter_param = 0.15;
    //赋值
    bool flag = false;
    lock_ee.lock();
    Wrench[0] = fs_filter_param*wrench_ee[0]+(1-fs_filter_param)*wrench_ee_last[0]-fsh_bias_ee[0];
    Wrench[1] = fs_filter_param*wrench_ee[1]+(1-fs_filter_param)*wrench_ee_last[1]-fsh_bias_ee[1];
    Wrench[2] = fs_filter_param*wrench_ee[2]+(1-fs_filter_param)*wrench_ee_last[2]-fsh_bias_ee[2];
    Wrench[3] = fs_filter_param*wrench_ee[3]+(1-fs_filter_param)*wrench_ee_last[3]-fsh_bias_ee[3];
    Wrench[4] = fs_filter_param*wrench_ee[4]+(1-fs_filter_param)*wrench_ee_last[4]-fsh_bias_ee[4];
    Wrench[5] = fs_filter_param*wrench_ee[5]+(1-fs_filter_param)*wrench_ee_last[5]-fsh_bias_ee[5];
    // printf("Wrench[1] = %2f;    Wrench[2] = %2f;\n",Wrench[1],Wrench[2]);
    flag = true;
    lock_ee.unlock();

    wrench_ee_last[0] = wrench_ee[0];
    wrench_ee_last[1] = wrench_ee[1];
    wrench_ee_last[2] = wrench_ee[2];
    wrench_ee_last[3] = wrench_ee[3];
    wrench_ee_last[4] = wrench_ee[4];
    wrench_ee_last[5] = wrench_ee[5];




    return flag;
}
static int count = 0;
/*
工作线程
*/
int DualFoSenDriver::FoSenWorkThread(char* device,float* wwrench,std::mutex& Fo_mutex)
{

    //2. 打开串口
        //std::cout<<serialName_ee<<std::endl;
    usleep(20000);
    std::cout<<device<<std::endl;
    int Serialhandle = open(device,O_RDWR | O_NOCTTY |O_NONBLOCK);
    printf("WriteData111 fd = %d \n",Serialhandle);
    float Wrench[6];

    unsigned char data[28];
    unsigned char t[4];
    union xxx{char m[4]; float n;}z;

    //缓存中的数据buffer
    std::deque<unsigned char> receiveddata;

    if(Serialhandle<0)
    {
        printf("Open serial port /dev/ttyUSB failed!\n");
        return -1;
    }
    else{
      printf("Open serial port /dev/ttyUSB success!\n");
    }

    struct termios OnesensorTermios_old;
    struct termios OnesensorTermios;

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


            // for(int i=0;i<m_rec_data_length;i++)
            // printf(" %d ",m_rec_data[i]);
            // printf("\n");
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
                //  printf("BIT 222:Fy= %2f Kg\n",z.n);
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

                Fo_mutex.lock();
                memcpy(wwrench,Wrench,sizeof(Wrench));
                Fo_mutex.unlock();
                // if(count++ == 20)
                // {
                // printf("BIT 111:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",Wrench[0],Wrench[1],Wrench[2],Wrench[3],Wrench[4],Wrench[5]);
                //     printf("BIT 222:Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",wwrench[0],wwrench[1],wwrench[2],wwrench[3],wwrench[4],wwrench[5]);
                //     count = 0;
                // }

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


int main(int argc, char* argv[])
{
    //./devel/lib/diana_driver/dualFoSen_node "/dev/ttyUSB0" "/dev/ttyUSB1"
    char* serialName_ee = argv[1];
    char* serialName_hh = argv[2];



    Eigen::Matrix<double, 6, 1> wrench_hh;
    Eigen::Matrix<double, 6, 1> wrench_ee;

    DualFoSenDriver sensor_layer(serialName_ee,serialName_hh);

    sleep(4);
    sensor_layer.setZeroPoint();

    while(1)
    {
        sensor_layer.getWrenchee(wrench_ee);
        std::cout<<"EE Wrench = "<<wrench_ee.transpose()<<std::endl;
        sensor_layer.getWrenchhh(wrench_hh);
        std::cout<<"HH Wrench = "<<wrench_hh.transpose()<<std::endl;

        // usleep(20000);

    }
}
