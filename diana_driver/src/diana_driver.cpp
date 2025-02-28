#include "diana_driver.hpp"

using namespace std;

DianaDriver* DianaDriver::CurMy = NULL;

/*
机械臂的初始化函数：
输入： (ip机械臂,串口地址ee,串口地址hh,ip上位机)； 例：“192.168.10.10” ，'dev/ttyUSB0'
./diana_driver_node '192.168.10.11' '/dev/ttyUSB0' '/dev/ttyUSB1' '0.0.0.0'
*/
DianaDriver::DianaDriver(char* ip_name)
{
    //1. 初始化消息指针
    printf("Init Server...1\n");


    pinfo = new srv_net_st();
    memset(pinfo ->SrvIp, 0x00, sizeof(pinfo ->SrvIp));
    memcpy(pinfo ->SrvIp, ip_name, strlen(ip_name));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;

    _ip_name = ip_name;
    memset(&EquilibriumPoint, 0x00, sizeof(EquilibriumPoint));
    memset(&speed, 0x00, sizeof(speed));
    
    memset(&robot_state, 0x00, sizeof(robot_state));
    mode_robot = T_MODE_NONE;

    startDiana();

}

/*
机械臂的析构函数：
当对象被销毁时自动调用，用于回收通信资源，保证机械臂安全
输入：None
*/
DianaDriver::~DianaDriver()
{
    int ret = destroySrv();
    if(ret < 0)
    {
        printf("destroySrv failed! Return value = %d\n", ret);
    }

    //Close();

}


void DianaDriver::wait_move()
{
    sleep(1.0);
    while (true)
    {
        const char state = getRobotState();
        //std::cout<<"state = "<< int(state)<<std::endl;
        if (state != 0)
            break;
        else
            sleep(0.1);
    }
    stop();
}


void DianaDriver::setCurMy()
{//设置当前对象为回调函数调用的对象  
    CurMy = this;  
}

void DianaDriver::_StateFeedback(StrRobotStateInfo *pinfo)
{
    CurMy->StateFeedback(pinfo);
}

void DianaDriver::_errorControl(int err)
{
    CurMy->errorControl(err);
}



/*
机械臂与上位机通信初始化函数,阻塞函数
*/
void DianaDriver::startDiana()
{

    /*初始化机械臂*/
    setCurMy();



    // void (DianaDriver::*ptrStrRobotStateInfo)(StrRobotStateInfo* ) = &DianaDriver::StateFeedback;
    // void (DianaDriver::*errorControl)(int) = &DianaDriver::errorControl;
    printf("Run to here!!! 11122\n");
    // if(initSrv(std::bind(&DianaDriver::errorControl,this,std::placeholders::_1), std::bind(&DianaDriver::StateFeedback,this,std::placeholders::_1), pinfo)){
    if(initSrv(_errorControl,_StateFeedback,pinfo)){
        printf("[Info] Fail to initialize Diana robot !\n");
        exit(-1);
    }
    else{
        printf("[Info] Initialize Diana robot successfully!\n");
    }


    double a[7],alpha[7],d[7],theta[7];
    int ret = getDH(a,alpha,d,theta);
    if(ret < 0)
    {
        printf("getDH failed!\n");
    }
    std::cout<<"a = "<<a[0]<<", "<<a[1]<<", "<<a[2]<<", "<<a[3]<<", "<<a[4]<<", "<<a[5]<<", "<<a[6]<<";"<<std::endl;
    std::cout<<"alpha = "<<alpha[0]<<", "<<alpha[1]<<", "<<alpha[2]<<", "<<alpha[3]<<", "<<alpha[4]<<", "<<alpha[5]<<", "<<alpha[6]<<";"<<std::endl;
    std::cout<<"d = "<<d[0]<<", "<<d[1]<<", "<<d[2]<<", "<<d[3]<<", "<<d[4]<<", "<<d[5]<<", "<<d[6]<<";"<<std::endl;
    std::cout<<"theta = "<<theta[0]<<", "<<theta[1]<<", "<<theta[2]<<", "<<theta[3]<<", "<<theta[4]<<", "<<theta[5]<<", "<<theta[6]<<";"<<std::endl;

    ret = setCollisionLevel(1);
    if(ret < 0)
    {
        printf("setCollision failed!\n");
    }

    double collision[7] = {200, 200, 200, 200, 200, 200, 200};
    if (setJointCollision(collision) < 0)
    {
        printf("Diana API setJointCollision failed!\n");
    }

    double arrStiff[6] = { 6000, 6000, 6000, 1000, 1000, 1000};
    double arrDamp[6] = { 120, 120, 120, 10, 10, 10};
    if (setCartImpedance(arrStiff, arrDamp) < 0)
    {
        printf("Diana API setCartImpedance failed!\n");
    }

    double dblPayload[10] = {1.2,//1.2, //mass 0.8
                            0.063,
                            -0.027,
                            -0.0508,    //point along z axis 0.05
                            950*1e-6,
                            -240*1e-6,
                            -241*1e-6,
                            1374*1e-6,
                            -159*1e-6,
                            66*1e-6};

    ret = setActiveTcpPayload(dblPayload);
    if(ret<0)
    {
        printf("Set error\n");
    }

}



/*
状态返回函数：
每调用1000次，打印一次关节角、电流和力矩，用于显示
输入： Robot feedback struct pinfo
*/
void DianaDriver::StateFeedback(StrRobotStateInfo *pinfo)
{

    //gettimeofday( &start, NULL );
    // if(pinfo){
    //     printf("Run to HERE 111 staCnt =  %d\n",staCnt);

    // }
    lock_robot.lock();
    memcpy(&robot_state, pinfo, sizeof(robot_state));
    lock_robot.unlock();
/******************************************************************/
/******************************************************************/
    /*通信测试*/
    // 1. 打印各个变量，测试速度。大约1khz左右
    static int staCnt = 1;
    static struct timeval start;
    static struct timeval end;
    // if(staCnt++ % 1000 == 0 && pinfo) 
    // {

    //     gettimeofday(&end, NULL);
    //     double timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec; 
    //     timeuse /= 1000000;
    //     printf("timeuse = %f\n",timeuse);
    //     memcpy(&start, &end, sizeof(end));
    //     for(int i=0; i<7; ++i)
    //     {
    //         printf("jointCurrent [%d] = %f \n", i, pinfo-> jointCurrent [i]);
    //         printf("jointTorque [%d] = %f \n", i, pinfo-> jointTorque [i]);

    //         if(i<6)
    //         {
    //             printf("tcpPos [%d] = %f \n",i,pinfo->tcpPos[i]);
    //         }

    //     }
    // }
/******************************************************************/
/******************************************************************/
    // 2. 控制层


}

/*
抱闸开启函数
*/
int DianaDriver::DianaReleaseBrake()
{
    int ret = releaseBrake();
    if(ret<0)
    {
        printf("releaseBrake failed! Return value = %d\n", ret);
    }
    sleep(2.0);
    return ret;
}


/*
抱闸关闭函数
*/
int DianaDriver::DianaholdBrake()
{
    int ret = holdBrake();
    if(ret<0)
    {
        printf("releaseBrake failed! Return value = %d\n", ret);
    }
    sleep(0.5);
    return ret;
}

void DianaDriver::DianaStop()
{
    stop();
}

int DianaDriver::getPoseBsToTCP(Eigen::Matrix<double, 4, 4>& T)
{
    double poses[6] = {0.0};
    int ret = getTcpPos(poses);
    if(ret < 0)
    {
        printf("getTcpPos failed! Return value = %d\n", ret);
    }

    double matrix[16]={0.0};
    ret =pose2Homogeneous(poses,matrix);

    T= Eigen::Map<Eigen::Matrix<double, 4, 4>>(matrix);
    //std::cout<<T<<std::endl;

  return 0;  
}

/*
抱闸关闭函数
*/
void DianaDriver::SwitchRTMode(mode_c mode)
{
    lock_robot_mode.lock();
    mode_robot =mode;

    lock_robot_mode.unlock();

    // double* ptrTCP = nullptr;
    // double TCPMatrix[16] = {1.0,0.0,0.0,0.0,
    //                          0.0,1.0,0.0,0.0,
    //                          0.0,0.0,1.0,0.0,
    //                          0.0,0.0,0.0,1.0};
    // if (mode ==T_MODE_RT){
    //     ptrTCP = TCPMatrix;
    // }
    // else{
    //     ptrTCP = nullptr;
    // }

    // int ret = setDefaultActiveTcp(ptrTCP);
    // if(ret < 0)
    // {
    //     printf("setDefaultActiveTcp failed! Return value = %d\n", ret);
    // }
    // sleep(1.0);
}

mode_c DianaDriver::getControlMode()
{
    
    lock_robot_mode.lock();
    mode_c mode = mode_robot;
    lock_robot_mode.unlock();

    return mode;
}




/*
错误返回函数：
用于返回错误，见思灵C++ 1. initSrv 部分例子介绍
输入： Robot feedback struct pinfo
*/
void DianaDriver::errorControl(int e)
{
    const char * strError = formatError(e); //该函数后面会介绍
    printf("error code (%d):%s\n", e, strError);
}


// void DianaDriver::RTcontrolThread()
// {

// }

int DianaDriver::setEquilibriumPoint(const Eigen::Matrix<double, 6, 1> pose)
{
    
    lock_controller.lock();

    memcpy(EquilibriumPoint,pose.data(),sizeof(EquilibriumPoint));

    int ret = servoL_ex (EquilibriumPoint, 0.02, 0.1, 500, 0.8, false);

    //cout<<EquilibriumPoint[0]<<" "<<EquilibriumPoint[1]<<"\n"<<endl;

    lock_controller.unlock();
    //cout<<"EquilibriumPoint ="<<pose.transpose()<<endl;

    return ret;

}

int DianaDriver::setVel(const Eigen::Matrix<double, 6, 1> vel)
{
    
    lock_controller.lock();

    memcpy(speed,vel.data(),sizeof(EquilibriumPoint));

    double acc[2] = {0.80, 0.80};

    int ret = speedL_ex(speed, acc, 0,true, nullptr);

    //cout<<EquilibriumPoint[0]<<" "<<EquilibriumPoint[1]<<"\n"<<endl;

    lock_controller.unlock();
    //cout<<"EquilibriumPoint ="<<pose.transpose()<<endl;

    return ret;

}

int DianaDriver::getCurrentTCPPose(Eigen::Matrix<double, 6, 1>& pose)
{
    double poses[6] = {0.0};
    int ret = getTcpPos(poses);
    if(ret < 0)
    {
        printf("getTcpPos failed! Return value = %d\n", ret);
    }
    else{
        // memcpy(pose.data(),&poses,sizeof(poses));
        pose[0]=poses[0];
        pose[1]=poses[1];
        pose[2]=poses[2];
        pose[3]=poses[3];
        pose[4]=poses[4];
        pose[5]=poses[5];
        // printf("123456789\n");
    }
    //cout<<pose[0]<<" "<<pose[1]<<"\n"<<endl;
    return ret;
}

int DianaDriver::DIANAmoveJToPose(Eigen::Matrix<double,6,1> target)
{
    double* poses = target.data();
    std::cout<<"poses = "<< target<<std::endl;
    double vel = 0.1;
    double acc = 0.2;

    int ret1 = moveL(poses, vel, acc, nullptr);

    if(ret1 < 0)
    {
        printf("moveJToPose failed! Return value = %d\n", ret1);
    }
    wait_move();
}

int DianaDriver::getJointState(Eigen::Matrix<double,7,1>& joint_state)
{

    double joints[7] = {0.0};
    int ret = getJointPos(joints);
    if(ret < 0)
    {
    printf("getJointPos failed! Return value = %d\n", ret);
    }

    joint_state(0) = joints[0];
    joint_state(1) = joints[1];
    joint_state(2) = joints[2];
    joint_state(3) = joints[3];
    joint_state(4) = joints[4];
    joint_state(5) = joints[5];
    joint_state(6) = joints[6];

}

int DianaDriver::DIANAchangeControlMode(mode_e m)
{
    printf("RUN TO HERE 111！\n");
    int ret = changeControlMode(m);
    printf("RUN TO HERE 1112！\n");
    if(ret < 0)
    {
        printf("changeControlMode failed! Return value = %d\n", ret);
        return -1;
    }
    else{
        printf("changeControlMode Success! impedance control = %d\n", ret);
        return 1;
    }
}


int main(int argc, char* argv[])
{
    char* ip_address_robot = argv[1];


    // Eigen::Matrix<double, 7, 1> wrench_hh;
    Eigen::Matrix<double, 6, 1> pose;
    Eigen::Matrix<double, 6, 1> pose_current;

    Eigen::Matrix<double, 6, 1> delta;
    pose.setZero();
    pose_current.setZero();
    delta.setZero();

    DianaDriver robot_layer(ip_address_robot);

/*************************************************/
/*****************STEP TEST**********************/
/*************************************************/
    int flag = 0;
    printf("Hello1\n");
    robot_layer.DianaReleaseBrake();
    sleep(3);
    char ch = {};
    int ret = -1;
    mode_e m = T_MODE_CART_IMPEDANCE;
    ret = robot_layer.DIANAchangeControlMode(m);
    printf("Press a putton to move the robot (a:go ahead, b:go back,Esc:Exit safely)\n");
    while(1)
    {
        ch = cin.get();//使用_getch()获取按下的键值
        if (ch == 27)
        { 
            break; 
        }//当按下ESC时退出循环，ESC键的键值是27.
        //pose[0] = pose[0]+0.1;
        ret = robot_layer.getCurrentTCPPose(pose_current);

        delta<<0.1,0.0,0.0,0.0,0.0,0.0;

        pose = pose_current+delta;
        robot_layer.setEquilibriumPoint(pose);
        std::cout<<"ret = "<<ret<<std::endl;
        //robot_layer.getWrenchee(wrench_ee);
        //std::cout<<"EE Wrench = \n"<<wrench_ee<<std::endl;
        //robot_layer.getWrenchhh(wrench_hh);
        //std::cout<<"HH Wrench = \n"<<wrench_hh<<std::endl;
        // printf("Hello2\n");
        // sleep(3);


        
	

    }
    robot_layer.DianaholdBrake();

/*************************************************/
/*****************STEP TEST_END**********************/
/*************************************************/
    return 0;
}











