#include "CameraComm_Server.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;

void CameraComm_Server::addfd(int epollfd, int fd, bool enable_et)
{
    struct epoll_event ev;
    ev.data.fd = fd;
    ev.events = EPOLLIN;
    if( enable_et )
        ev.events = EPOLLIN;
        //ev.events = EPOLLIN | EPOLLET;
    epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev);
    // 设置socket为nonblocking模式
    // 执行完就转向下一条指令，不管函数有没有返回。
    //fcntl(fd, F_SETFL, fcntl(fd, F_GETFD, 0)| O_NONBLOCK);
    printf("fd added to epoll!\n\n");
}

/*
机械臂的初始化函数：
输入： (ip机械臂,串口地址ee,串口地址hh,ip上位机)； 例：“192.168.10.10” ，'dev/ttyUSB0'
./diana_driver_node '192.168.10.11' '/dev/ttyUSB0' '/dev/ttyUSB1' '0.0.0.0'
*/
CameraComm_Server::CameraComm_Server(char* upper_computer)
{

    _upper_computer = upper_computer;

    // 初始化服务器地址和端口
    serverAddr.sin_family = PF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    printf("Init Server...2\n");
    
    serverAddr.sin_addr.s_addr = inet_addr(upper_computer);
    printf("Init Server...3\n");
    // 初始化socket
    listener = 0;
    
    // epool fd
    epfd = 0;

    /*初始化上位机TCP连接*/
    printf("Init Server...\n");

    //创建监听socket
    listener = socket(PF_INET,SOCK_STREAM, 0);
    if(listener < 0) 
    {
        perror("listener"); 
        exit(-1);
    }
    int reuse = 0;

    if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
            perror("setsockopet error\n");
            exit(-1);
    }


    //绑定地址
    if( bind(listener, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind error");
        exit(-1);
    }

    //监听
    int ret = listen(listener, 5);
    if(ret < 0) 
    {
        perror("listen error"); 
        exit(-1);
    }

    std::cout << "Start to listen: " << upper_computer << std::endl;

    // //在内核中创建事件表 epfd是一个句柄 
    // epfd = epoll_create (EPOLL_SIZE);
    
    // if(epfd < 0) {
    //     perror("epfd error");
    //     exit(-1);
    // }
 
    // //往事件表里添加监听事件
    // addfd(epfd, listener, true);


    lock_cam.lock();
    memset(&sub_message,0,sizeof(sub_message));
    memset(&pub_message,0,sizeof(pub_message));
    lock_cam.unlock();

    thread_cam = std::thread(std::mem_fn(&CameraComm_Server::CamWorkThread),this);
}


CameraComm_Server::~CameraComm_Server()
{
    thread_cam.detach();
    Close();

}



void CameraComm_Server::CamWorkThread()
{
    char recv_buf_static[BUF_SIZE];
    /*初始化TCP*/
    //主循环
    // static struct epoll_event events[EPOLL_SIZE];
    //     //epoll_events_count表示就绪事件的数目
    // int epoll_events_count = epoll_wait(epfd, events, EPOLL_SIZE, -1);

    // if(epoll_events_count < 0) {
    //     perror("epoll failure");
    //     exit(-1);
    // }

    // std::cout << "epoll_events_count =\n" << epoll_events_count << std::endl;
    // int sockfd = events[0].data.fd;

    // struct sockaddr_in client_address;
    // socklen_t client_addrLength = sizeof(struct sockaddr_in);
    // int clientfd = accept(listener, ( struct sockaddr* )&client_address, &client_addrLength );



    // if(clientfd<0)
    // {
    //     exit(1);
    // }
    // else{
    // cout << "client connection from: "
    //         << inet_ntoa(client_address.sin_addr) << ":"
    //         << ntohs(client_address.sin_port) << ", clientfd = "
    //         << clientfd << endl;

    // }

    // addfd(epfd, clientfd, true);
    // int nrecvSize =0; 
    int connfd;
            
    int ret;
    
    if( (connfd = accept(listener, (struct sockaddr*)NULL, NULL)) == -1)
    {
            printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
            //continue;
    }
    // std::cout<<"111!!!@@@@!!!!!!!!!!!!!!!!!"<<std::endl;
    
    while(1)
    {


        // std::cout<<"222!!!@@@@!!!!!!!!!!!!!!!!!"<<std::endl;
        ret = SendMsgToUpper(connfd);
        //std::cout<<"buff2"<<std::endl;
        if(ret < 0) {
            perror("error");
            Close();
            exit(-1);
        }
        // std::cout<<"333!!!@@@@!!!!!!!!!!!!!!!!!"<<std::endl;

        


        memset(&pub_message,0,sizeof(pub_message));
        // std::cout<<"444!!!@@@@!!!!!!!!!!!!!!!!!"<<std::endl;

        ret = recv(connfd, recv_buf_static, BUF_SIZE, 0);

        lock_cam.lock();
        // std::cout<<"buff"<<recv_buf_static<<std::endl;
        //清空结构体
        memset(&pub_message,0,sizeof(pub_message));
        //将发来的消息转换为结构体
        memcpy(&pub_message,recv_buf_static,sizeof(pub_message));
        //std::cout<<"buff1"<<std::endl;
        lock_cam.unlock();



    }
    
 
    // 关闭服务
    //Close();    

}


/*
关闭服务器，退出TCP
*/
void CameraComm_Server::Close() {
 
    //关闭socket
    close(listener); 
    //关闭epoll监听
    close(epfd);
}

/*
发送信息到上位机
TODO：将机器人信息发送出去
*/
int CameraComm_Server::SendMsgToUpper(int clientfd)
{

    char send_buf[BUF_SIZE];
    char recv_buf[BUF_SIZE];
    // memset(recv_buf,0,sizeof(recv_buf));
    // int len = recv(clientfd, recv_buf, sizeof(recv_buf),0);
    usleep(10000);


    //清空结构体，把接收到的字符串转换为结构体
    // memset(&sub_message,0,sizeof(sub_message));
    // memcpy(&sub_message,recv_buf,sizeof(sub_message));
    bzero(send_buf, BUF_SIZE);

    std::cout<<"sub_message "<<sub_message.flag<<std::endl;

    lock_cam.lock();
    memcpy(send_buf,&sub_message,sizeof(sub_message));
    lock_cam.unlock();

    int ret = send(clientfd, send_buf, sizeof(sub_message), 0);


    //清空发送缓存

    return ret;
}


int CameraComm_Server::getRegistInfoHandEye(Eigen::Matrix<double, 7, 1>& regist_info_handeye)
{
    lock_cam.lock();
    //
    regist_info_handeye[0]=sub_message.hand_eye_registation_pos[0];
    regist_info_handeye[1]=sub_message.hand_eye_registation_pos[1];
    regist_info_handeye[2]=sub_message.hand_eye_registation_pos[2];
    regist_info_handeye[3]=sub_message.hand_eye_registation_ori[0];
    regist_info_handeye[4]=sub_message.hand_eye_registation_ori[1];
    regist_info_handeye[5]=sub_message.hand_eye_registation_ori[2];
    regist_info_handeye[6]=sub_message.hand_eye_registation_ori[3];

    lock_cam.unlock();
    return 1;
}

int CameraComm_Server::getJoint(Eigen::Matrix<double, 6, 1>& joint)
{
    lock_cam.lock();
    //
    joint[0]=pub_message.joint[0];
    joint[1]=pub_message.joint[1];
    joint[2]=pub_message.joint[2];
    joint[3]=pub_message.joint[3];
    joint[4]=pub_message.joint[4];
    joint[5]=pub_message.joint[5];

    lock_cam.unlock();
    return 1;
}

int CameraComm_Server::getMarkerPose(Eigen::Matrix<double, 7, 1>& pose)
{
    lock_cam.lock();
    //
    pose[0]=pub_message.marker_pos[0];
    pose[1]=pub_message.marker_pos[1];
    pose[2]=pub_message.marker_pos[2];
    pose[3]=pub_message.marker_ori[0];
    pose[4]=pub_message.marker_ori[1];
    pose[5]=pub_message.marker_ori[2];
    pose[6]=pub_message.marker_ori[3];

    lock_cam.unlock();
    return 1;
}


int CameraComm_Server::getRegistObject(Eigen::Matrix<double, 7, 1>& regist_info_object)
{
    lock_cam.lock();
    //
    regist_info_object[0]=sub_message.object_registation_pos[0];
    regist_info_object[1]=sub_message.object_registation_pos[1];
    regist_info_object[2]=sub_message.object_registation_pos[2];
    regist_info_object[3]=sub_message.object_registation_ori[0];
    regist_info_object[4]=sub_message.object_registation_ori[1];
    regist_info_object[5]=sub_message.object_registation_ori[2];
    regist_info_object[6]=sub_message.object_registation_ori[3];

    lock_cam.unlock();
    return 1;
}


int CameraComm_Server::setRegistInfoHandEye(const Eigen::Matrix<double, 7, 1> regist_info_handeye)
{
    lock_cam.lock();
    //
    sub_message.hand_eye_registation_pos[0]=regist_info_handeye[0];
    sub_message.hand_eye_registation_pos[1]=regist_info_handeye[1];
    sub_message.hand_eye_registation_pos[2]=regist_info_handeye[2];
    sub_message.hand_eye_registation_ori[0]=regist_info_handeye[3];
    sub_message.hand_eye_registation_ori[1]=regist_info_handeye[4];
    sub_message.hand_eye_registation_ori[2]=regist_info_handeye[5];
    sub_message.hand_eye_registation_ori[3]=regist_info_handeye[6];

    sub_message.flag = 1;

    lock_cam.unlock();
    return 1;
}


int CameraComm_Server::setRegistObject(const Eigen::Matrix<double, 7, 1> regist_info_object)
{
    lock_cam.lock();
    //
    sub_message.object_registation_pos[0]=regist_info_object[0];
    sub_message.object_registation_pos[1]=regist_info_object[1];
    sub_message.object_registation_pos[2]=regist_info_object[2];
    sub_message.object_registation_ori[0]=regist_info_object[3];
    sub_message.object_registation_ori[1]=regist_info_object[4];
    sub_message.object_registation_ori[2]=regist_info_object[5];
    sub_message.object_registation_ori[3]=regist_info_object[6];


    sub_message.flag = 1;

    lock_cam.unlock();
    return 1;
}






int main(int argc, char* argv[])
{
    //./devel/lib/diana_driver/dualFoSen_node "/dev/ttyUSB0" "/dev/ttyUSB1"
    char* tcp_ip = argv[1];
//Eigen::Matrix<double, 4, 4> T;
// T<<-0.336802,  0.822082,  0.459069,  0.172824,
// -0.455932,  0.284193, -0.843422, -0.580309,
// 0.823826,  0.493371, -0.279097, -0.200922,
// 0,         0,         0,         1;


// Eigen::Matrix<double, 6, 1> regist_inform_object_rr;

// TransformToPose(T,regist_inform_object_rr);


    Eigen::Matrix<double, 7, 1> regist_inform_handeye;
    Eigen::Matrix<double, 7, 1> regist_inform_object;
    Eigen::Matrix<double, 7, 1> marker_pose;
    regist_inform_handeye.setZero();
    regist_inform_object.setZero();
    marker_pose.setZero();

    CameraComm_Server camsensor_layer(tcp_ip);
    double iter = 0.0;


    Eigen::Matrix<double,6,1> regist_inform_object_rpy;

    Eigen::Matrix<double,4,4> T_temp;
    Eigen::Matrix<double,6,1> pose_base_mkr;

    YAML::Node config_base_mkr = YAML::LoadFile("/home/agr/catkin_ws/src/diana_driver/path/configBaseToMkr.yaml");


    pose_base_mkr<<config_base_mkr["x"].as<double>(), 
                config_base_mkr["y"].as<double>(), 
                config_base_mkr["z"].as<double>(), 
                config_base_mkr["rx"].as<double>(), 
                config_base_mkr["ry"].as<double>(), 
                config_base_mkr["rz"].as<double>();

    while(1)
    {
        iter = iter+0.01;
        regist_inform_handeye[0] = 0.0;
        // regist_inform_handeye =
        // regist_inform_object[0] = 0.0;
        // regist_inform_object[1] = 0.0;
        // regist_inform_object[2] = 0.0;
        // regist_inform_object<<0.172824, -0.580309, -0.200922,  0.559777, -0.217028, -0.788226,  0.135087;

        //regist_inform_object<< 0.0632775, -0.124566,   0.25637,  0.283269,  0.448894,   0.14808,  0.834461;
        // regist_inform_object<< 0.104428, -0.675247,  0.575827,  0.138936,  0.982376,  0.113061, 0.0533899;
         //regist_inform_object<<0.124161, -0.0388021,    0.61513,   0.538621,  -0.235888,   0.688753,   0.424102;
         //regist_inform_object<<-0.134918,  -1.08171,  0.347825,  0.352319,  0.291848, -0.834678,  0.30661;
        //  regist_inform_object<<-0.102099,  -1.18057,   0.34885,  0.579076,  0.158502, -0.553275,  0.577439;
         //regist_inform_object<<0.630274, -0.772932,  0.399078,  0.297859,  0.521279, 0.0163683,   0.79955;
         
         
        //  regist_inform_object<<-0.530344, -0.427285,  0.399078,  0.521339, -0.297753, -0.799547, 0.0165311;
// 0.587542  -0.192224   0.434491   0.932812 -0.0151513 -0.0355094   -0.35829
        // regist_inform_object<< 0.612591,  -0.171981,   0.407329,   0.914944,  0.0356169, 0.00579995,  -0.401965;
        // regist_inform_object_rpy<<0.550048, -0.34,       0.4,0.0,0.0,-0.4+1.4;

        // regist_inform_object_rpy<<0.370048, -0.50,       0.2345,0.0,0.0,-0.4+1.4;
        //regist_inform_object_rpy<<0.370048, -0.50,       0.2345,0.0,0.0,1.4;

        // regist_inform_object_rpy<<0.370048+0.0025*sin(iter*2*M_PI), -0.50,       0.3345,0.0,0.0,1.2+1.6;
        // regist_inform_object_rpy<<0.500048, -0.70,  0.4045,0.0,0.0,2.8;

        // regist_inform_object_rpy<<0.516439, -0.54492, 0.168912, 0.0995577, -0.0247797, 1.57492;
        regist_inform_object_rpy = pose_base_mkr;

        //Tracking Test
        // regist_inform_object_rpy<<0.370048, -0.50+0.002*sin(iter*0.1*2*M_PI),       0.2845,0.0,0.0,1.6;
        
        
        // regist_inform_object_rpy<<0.370048, -0.50,       0.2845,0.0,0.0,1.6;
        EulerPoseRPY2QuatPose(regist_inform_object_rpy,regist_inform_object);



        // regist_inform_object<<0.550048, -0.339953,       0.4,  0.921657,         0,         0, -0.388005;
        //  regist_inform_object<<0.172824, 0.0303092,   0.299078,  -0.158041,   0.579202,   0.576998,   0.553735;
        //0.0632775 -0.124566   0.25637  0.283269  0.448894   0.14808  0.834461
        camsensor_layer.setRegistObject(regist_inform_object);
        // std::cout<<"regist_inform_object = \n"<<regist_inform_object.transpose()<<std::endl;
        camsensor_layer.setRegistInfoHandEye(regist_inform_handeye);
        // std::cout<<"regist_inform_handeye = \n"<<regist_inform_handeye.transpose()<<std::endl;

        camsensor_layer.getMarkerPose(marker_pose);
        // std::cout<<"marker_pose = \n"<<marker_pose.transpose()<<std::endl;
        // std::cout<<"iter = "<<iter<<std::endl;
        

        usleep(100000);

    }
}
