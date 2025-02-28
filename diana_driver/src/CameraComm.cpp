#include "CameraComm.hpp"

using namespace std;

void CameraComm::addfd(int epollfd, int fd, bool enable_et)
{
    struct epoll_event ev;
    ev.data.fd = fd;
    ev.events = EPOLLIN;
    if( enable_et )
        ev.events = EPOLLIN | EPOLLET;
    epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev);
    // 设置socket为nonblocking模式
    // 执行完就转向下一条指令，不管函数有没有返回。
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFD, 0)| O_NONBLOCK);
    printf("fd added to epoll!\n\n");
}

/*
机械臂的初始化函数：
输入： (ip机械臂,串口地址ee,串口地址hh,ip上位机)； 例：“192.168.10.10” ，'dev/ttyUSB0'
./diana_driver_node '192.168.10.11' '/dev/ttyUSB0' '/dev/ttyUSB1' '0.0.0.0'
*/
CameraComm::CameraComm(char* upper_computer)
{

    _upper_computer = upper_computer;

    // 初始化服务器地址和端口
    serverAddr.sin_family = PF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(upper_computer);
    printf("Init Server...2\n");
    
    printf("Init Server...3\n");
    // 初始化socket
    sock = 0;
    pid = 0;
    isClientwork = true;
    
    // epool fd
    epfd = 0;

    /*初始化上位机TCP连接*/
    cout << "Connect Server: " << upper_computer << " : " << SERVER_PORT << endl;
        

    //创建socket
    sock = socket(PF_INET,SOCK_STREAM, 0);
    if(sock < 0) 
    {
        perror("sock error"); 
        exit(-1);
    }

    // 连接服务端
    if(connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("connect error");
        exit(-1);
    }

    // 创建管道，其中fd[0]用于父进程读，fd[1]用于子进程写
    // if(pipe(pipe_fd) < 0) {
    //     perror("pipe error");
    //     exit(-1);
    // }

    //在内核中创建事件表 epfd是一个句柄 
    // epfd = epoll_create (EPOLL_SIZE);
    
    // if(epfd < 0) {
    //     perror("epfd error");
    //     exit(-1);
    // }
 
    // //往事件表里添加监听事件
    // addfd(epfd, sock, true);
    //addfd(epfd, pipe_fd[0], true);

    lock_cam.lock();
    memset(&sub_message,0,sizeof(sub_message));
    lock_cam.unlock();


    thread_cam = std::thread(std::mem_fn(&CameraComm::CamWorkThread),this);
    threadRT::setScheduling(thread_cam, SCHED_RR, 2);

}


CameraComm::~CameraComm()
{
    thread_cam.detach();
    if(isClientwork)
    {
        Close();
    }

}



void CameraComm::CamWorkThread()
{
    /*初始化TCP*/
    //主循环
    static struct epoll_event events[EPOLL_SIZE];
    char recv_buf[BUF_SIZE];

    while(isClientwork)
    {
        //std::cout<<"run to here run to here 1111"<<std::endl;
        int ret = recv(sock, recv_buf, BUF_SIZE, 0);
        if(ret < 0) {
            cout << "Server closed connection: " << sock << endl;
            isClientwork = 0;
        }
        //std::cout<<"run to here run to here 222"<<std::endl;
        
        for(int iter=0;iter<sizeof(sub_message);iter++)
        {
            char* p = recv_buf+sizeof(char)*iter; 
            // std::cout<<"[@@@@@@@@@cccccc]iter = "<<((Diana_Sub_Message* )p)->flag<<std::endl;        
            // memcpy(&pub_message_temp,(int* )recv_buf_static+iter*2,sizeof(pub_message_temp));
            if(((Diana_Sub_Message* )p)->flag ==1)
            {

                lock_cam.lock();
                memcpy(&sub_message,p,sizeof(Diana_Sub_Message));
                lock_cam.unlock();
                // std::cout<<"[@@@@@@@@@]iter = "<<iter<<std::endl;
                memset(&recv_buf,0,sizeof(recv_buf));
                // temp_c = recv_buf_static+iter*sizeof(char)+sizeof(Diana_Pub_Message);
                break;
            }
            
        }

        usleep(15000);
 
      
    }
 
    // 关闭服务
    Close();    

}


/*
关闭服务器，退出TCP
*/
void CameraComm::Close() {
 
    //关闭socket
    close(sock); 
    //关闭epoll监听
    close(epfd);
}

// /*
// 发送信息到上位机
// TODO：将机器人信息发送出去
// */
// int CameraComm::SendMsgToUpper(int clientfd)
// {
//     char recv_buf[BUF_SIZE];
//     char send_buf[BUF_SIZE];


//     bzero((char* )recv_buf, BUF_SIZE);
//     int len = recv(clientfd, recv_buf, BUF_SIZE, 0);
//     //清空结构体，把接收到的字符串转换为结构体
//     lock_cam.lock();
//     memset(&sub_message,0,sizeof(sub_message));
//     memcpy(&sub_message,recv_buf,sizeof(sub_message));
//     lock_cam.unlock();

//     //清空发送缓存
//     // bzero(send_buf, BUF_SIZE);
//     // memcpy(send_buf,&pub_message,sizeof(pub_message));
//     // send(clientfd, send_buf, sizeof(send_buf), 0);

//     return len;
// }

int CameraComm::SendMsgToUpper(Eigen::Matrix<double,7,1> marker_pose, Eigen::Matrix<double,7,1> joint)
{

    char send_buf[sizeof(Diana_Pub_Message)];
    usleep(500);

    Diana_Pub_Message pub_message;

    pub_message.marker_pos[0] = marker_pose(0);
    pub_message.marker_pos[1] = marker_pose(1);
    pub_message.marker_pos[2] = marker_pose(2);

    pub_message.marker_ori[0] = marker_pose(3);
    pub_message.marker_ori[1] = marker_pose(4);
    pub_message.marker_ori[2] = marker_pose(5);
    pub_message.marker_ori[3] = marker_pose(6);

    pub_message.joint[0] = joint(0);
    pub_message.joint[1] = joint(1);
    pub_message.joint[2] = joint(2);
    pub_message.joint[3] = joint(3);
    pub_message.joint[4] = joint(4);
    pub_message.joint[5] = joint(5);
    pub_message.joint[6] = joint(6);

    pub_message.flag = 1;

    //清空结构体，把接收到的字符串转换为结构体
    lock_cam.lock();
    bzero(send_buf, sizeof(Diana_Pub_Message));
    memcpy(send_buf,&pub_message,sizeof(pub_message));

    // std::cout<<"send_buf ="<<((Diana_Pub_Message*) send_buf)->flag<<std::endl;

    int ret = send(sock, send_buf, sizeof(send_buf), 0);
    // std::cout<<"send_buf 111="<<((Diana_Pub_Message*) send_buf)->flag<<std::endl;
    lock_cam.unlock();

    //清空发送缓存

    return ret;
}


int CameraComm::getRegistInfoHandEye(Eigen::Matrix<double, 7, 1>& regist_info_handeye)
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


int CameraComm::getRegistObject(Eigen::Matrix<double, 7, 1>& regist_info_object)
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

int CameraComm::getRegistInfoHandEye(Eigen::Matrix<double, 6, 1>& regist_info_handeye)
{
    lock_cam.lock();
    regist_info_handeye[0]=sub_message.object_registation_pos[0];
    regist_info_handeye[1]=sub_message.object_registation_pos[1];
    regist_info_handeye[2]=sub_message.object_registation_pos[2];

    //四原数转欧拉角

    double w = sub_message.object_registation_pos[3];
    double x = sub_message.object_registation_pos[4];
    double y = sub_message.object_registation_pos[5];
    double z = sub_message.object_registation_pos[6];
    Eigen::Quaterniond q(w,x,y,z);
    Eigen::Matrix3d rx = q.toRotationMatrix();
    Eigen::Vector3d ea = rx.eulerAngles(2,1,0);

    regist_info_handeye[3]= ea(0);
    regist_info_handeye[4]= ea(1);
    regist_info_handeye[5]= ea(2);


    lock_cam.unlock();
    return 1;
}
int CameraComm::getRegistObject(Eigen::Matrix<double, 6, 1>& regist_info_object)
{
    lock_cam.lock();
    regist_info_object[0]=sub_message.object_registation_pos[0];
    regist_info_object[1]=sub_message.object_registation_pos[1];
    regist_info_object[2]=sub_message.object_registation_pos[2];

    double w = sub_message.object_registation_pos[3];
    double x = sub_message.object_registation_pos[4];
    double y = sub_message.object_registation_pos[5];
    double z = sub_message.object_registation_pos[6];
    // Eigen::Quaterniond q(w,x,y,z);
    // Eigen::Vector3d ea = q.toRotationMatrix().eulerAngles(2,1,0);

    // regist_info_object[3]= ea(0);
    // regist_info_object[4]= ea(1);
    // regist_info_object[5]= ea(2);

    Eigen::Matrix<double,4,1> q;
    Eigen::Matrix<double,3,1> v;
    q<<w,x,y,z;

    Quat2AxisAngle(q,v);

    regist_info_object[3] = v[0];
    regist_info_object[4] = v[1];
    regist_info_object[5] = v[2];


    lock_cam.unlock();
    return 1;
}




int main(int argc, char* argv[])
{
    //./devel/lib/diana_driver/dualFoSen_node "/dev/ttyUSB0" "/dev/ttyUSB1"
    char* tcp_ip = argv[1];

    Eigen::Matrix<double, 7, 1> regist_inform_handeye;
    Eigen::Matrix<double, 7, 1> regist_inform_object;

    CameraComm camsensor_layer(tcp_ip);

    while(1)
    {
        camsensor_layer.getRegistObject(regist_inform_object);
        std::cout<<"regist_inform_object = \n"<<regist_inform_object<<std::endl;
        camsensor_layer.getRegistInfoHandEye(regist_inform_handeye);
        std::cout<<"regist_inform_handeye = \n"<<regist_inform_handeye<<std::endl;

        usleep(10000);

    }
}
