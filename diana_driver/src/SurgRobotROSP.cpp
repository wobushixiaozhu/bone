#include "ui_SurgRobotROS.h"
#include "SurgRobotROSP.hpp"

int poseToMsg(Eigen::Matrix<double,7,1> q, geometry_msgs::Pose& pose)
{
    pose.position.x = q(0);
    pose.position.y = q(1);
    pose.position.z = q(2);

    pose.orientation.w = q(3);
    pose.orientation.x = q(4);
    pose.orientation.y = q(5);
    pose.orientation.z = q(6);

    return 1;
}

int poseToTransformTF(Eigen::Matrix<double,7,1> pose_q,tf::Transform& transform)
{
    Eigen::Matrix<double,4,4> T_base_ee;
    Eigen::Matrix<double,6,1> pose;

    QuatPose2AxisAnglePose(pose_q,pose);
    PoseToTransform(pose,T_base_ee);


    // 把transform格式的转移矩阵转换为poseMsg.pose格式
    transform = tf::Transform(
                tf::Matrix3x3( T_base_ee(0, 0),T_base_ee(0, 1),T_base_ee(0, 2),
                                T_base_ee(1, 0),T_base_ee(1, 1),T_base_ee(1, 2),
                                T_base_ee(2, 0),T_base_ee(2, 1),T_base_ee(2, 2)),
                tf::Vector3(T_base_ee(0,3),T_base_ee(1,3),T_base_ee(2,3)));

    return 1;

}

int MatrixToTransformTF(Eigen::Matrix<double,4,4> T_base_ee,tf::Transform& transform)
{
    // Eigen::Matrix<double,4,4> T_base_ee;
    // Eigen::Matrix<double,6,1> pose;

    // QuatPose2AxisAnglePose(pose_q,pose);
    // PoseToTransform(pose,T_base_ee);


    // 把transform格式的转移矩阵转换为poseMsg.pose格式
    transform = tf::Transform(
                tf::Matrix3x3( T_base_ee(0, 0),T_base_ee(0, 1),T_base_ee(0, 2),
                                T_base_ee(1, 0),T_base_ee(1, 1),T_base_ee(1, 2),
                                T_base_ee(2, 0),T_base_ee(2, 1),T_base_ee(2, 2)),
                tf::Vector3(T_base_ee(0,3),T_base_ee(1,3),T_base_ee(2,3)));

    return 1;

}


int transformToVisMsg(tf::Transform transform,
                        std::string parent_link,
                        std::string child_link,
                        geometry_msgs::PoseStamped& poseMsg,
                        geometry_msgs::TransformStamped& transformMsg,
                    visualization_msgs::Marker& visMarker)
{
    	//stampTransform Msg
        ros::Time curr_stamp =ros::Time::now();
        // tf::StampedTransform* ptr = &stampedTransform
        tf::StampedTransform stampedTransform(transform, curr_stamp, 
                                            parent_link.c_str(), child_link.c_str());
        //将StampedTransform格式的信息发送出去
        //br.sendTransform(stampedTransform);

        //Gen poseMsg
        tf::poseTFToMsg(transform, poseMsg.pose);
        poseMsg.header.frame_id = parent_link.c_str();
        poseMsg.header.stamp = curr_stamp;


        //tf::transformStampedTFToMsg（输入，输出），是一种方法，相当于调用了一个函数
        tf::transformStampedTFToMsg(stampedTransform, transformMsg);
        transformMsg.header.frame_id = parent_link.c_str();
        transformMsg.child_frame_id = child_link.c_str();


        //Gen Vis Msg
        visMarker.header = transformMsg.header;
        visMarker.id = 2;
        visMarker.type = visualization_msgs::Marker::CUBE;
        visMarker.action = visualization_msgs::Marker::ADD;
        visMarker.pose = poseMsg.pose;
        visMarker.scale.x = 0.1;
        visMarker.scale.y = 0.1;
        visMarker.scale.z = 0.1;
        visMarker.color.r = 1.0;
        visMarker.color.g = 0;
        visMarker.color.b = 0;
        visMarker.color.a = 1.0;
        visMarker.lifetime = ros::Duration(3.0);

        return 1;

}

int JntToMsg(Eigen::Matrix<double,7,1> jnt_info,sensor_msgs::JointState& jointStatesMsg)
{
    jointStatesMsg.name.resize(7);
    jointStatesMsg.position.resize(7);

    ros::Time curr_stamp =ros::Time::now();
    jointStatesMsg.header.stamp = curr_stamp;

    jointStatesMsg.position[0] = jnt_info(0);
    jointStatesMsg.name[0] = "joint1";
    jointStatesMsg.position[1] = jnt_info(1);
    jointStatesMsg.name[1] = "joint2";
    jointStatesMsg.position[2] = jnt_info(2);
    jointStatesMsg.name[2] = "joint3";
    jointStatesMsg.position[3] = jnt_info(3);
    jointStatesMsg.name[3] = "joint4";
    jointStatesMsg.position[4] = jnt_info(4);
    jointStatesMsg.name[4] = "joint5";
    jointStatesMsg.position[5] = jnt_info(5);
    jointStatesMsg.name[5] = "joint6";
    jointStatesMsg.position[6] = jnt_info(6);
    jointStatesMsg.name[6] = "joint7";

    return 1;

}




SurgRobotROSP::SurgRobotROSP(QWidget *parent, int argc, char **argv):
                QWidget(parent),
                ui(new Ui::SurgRobotROS)
{
    ui->setupUi(this);
    QObject::connect(ui->RTcontrol, SIGNAL(clicked(void)),
                     this, SLOT(SurgRobotRTcontrol(void)));

    QObject::connect(ui->SCcontrol, SIGNAL(clicked(void)),
                    this, SLOT(SurgRobotSCcontrol(void)));

    QObject::connect(ui->Stop, SIGNAL(clicked(void)),
                    this, SLOT(SurgRobotStop(void)));

    std::cout<<"run to here1"<<std::endl;
    ros::init(argc,argv,"SurgRobotROS1");
    if (!ros::master::check())
    {
        ROS_INFO("No master started!");
        this->close();

    }
    ros::start();

    ros::NodeHandle n("~");

    pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = n.advertise<geometry_msgs::TransformStamped>("transform", 100);
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 4);

    marker_pub = n.advertise<visualization_msgs::Marker>("marker", 10);

    std::string ip_robot_str;
    std::string serial_ee_str;
    std::string serial_hh_str;
    std::string ip_upper_str;
    std::string path_str;


    n.param<std::string>("ip_robot",ip_robot_str,"192.168.10.75");
    n.param<std::string>("serial_ee",serial_ee_str,"/dev/hh_port");
    n.param<std::string>("serial_hh",serial_hh_str,"/dev/ee_port");
    n.param<std::string>("ip_upper",ip_upper_str,"127.0.0.1");
    n.param<std::string>("path",path_str,"/home/agr/catkin_ws/src/diana_driver/path/Spline_Plane.txt");

    char* ip_robot = const_cast<char *>(ip_robot_str.c_str());
    char* serial_ee = const_cast<char *>(serial_ee_str.c_str());
    char* serial_hh = const_cast<char *>(serial_hh_str.c_str());
    char* ip_upper = const_cast<char *>(ip_upper_str.c_str());
    char* path = const_cast<char *>(path_str.c_str());

    surgTask = new DianaAdmitControlPlaneCut(ip_robot,serial_ee,serial_hh,ip_upper,path);

    std::cout<<"run to here2"<<std::endl;
    ui->Stop->setDisabled(true);

    isRunning = true;

    thread_vis = std::thread(std::mem_fn(&SurgRobotROSP::SurgRobotVisThread),this);

}

SurgRobotROSP::~SurgRobotROSP()
{
    isRunning = false;
    sleep(0.1);
    thread_vis.detach();
    delete surgTask;

}

void SurgRobotROSP::SurgRobotVisThread(void)
{
    Eigen::Matrix<double, 7, 1> jnt_info;
    Eigen::Matrix<double, 7, 1> obj_info;
    Eigen::Matrix<double,6,1> ee_info_ax;
    Eigen::Matrix<double, 7, 1> ee_info;


    // 发送的几何量
    geometry_msgs::PoseStamped poseMsg;
    sensor_msgs::JointState jointStatesMsg;

    visualization_msgs::Marker visMarker;
    geometry_msgs::TransformStamped transformMsg;
    tf::Transform transform;
       

    
    jointStatesMsg.name.resize(7);
    jointStatesMsg.position.resize(7);

    jnt_info.setZero();
    obj_info.setZero();
    ee_info.setZero();
    
    tf::TransformBroadcaster br;
    while(isRunning)
    {
        //read data from tasks layer
        surgTask->getJointState(jnt_info);
        surgTask->getRegistObject(obj_info);
        surgTask->getCurrentTCPPose(ee_info_ax);

        AxisAnglePose2QuatPose(ee_info_ax,ee_info);

        //这个地方有问题
        surgTask->SendMsgToUpper(ee_info,jnt_info);

        // std::cout<<"jnt_info = "<<jnt_info.transpose()<<"\n"<<std::endl;

        //publish in ros
        Eigen::Matrix<double,4,4> T_obj;
        Eigen::Matrix<double,6,1> pose_obj;

        QuatPose2AxisAnglePose(obj_info,pose_obj);
        //std::cout<<"pose_obj = "<<pose_obj.transpose()<<"\n"<<std::endl;
        PoseToTransform(pose_obj,T_obj);

        Eigen::Matrix<double,4,4> T_plan_obj;
        surgTask->getPlanInfo(T_plan_obj);

        Eigen::Matrix<double,4,4> T_task2d = T_obj*T_plan_obj.inverse();

        std::cout<<"obj_info = "<<obj_info.transpose()<<"\n"<<std::endl;
        MatrixToTransformTF(T_task2d,transform);


        std::string parent_link("base_link");
        std::string child_link("marker_linkcc");

        ros::Time curr_stamp =ros::Time::now();

	    tf::StampedTransform stampedTransform(transform, curr_stamp, parent_link.c_str(), child_link.c_str());
        //将StampedTransform格式的信息发送出去
        br.sendTransform(stampedTransform);	


        //生成Msg 
        transformToVisMsg(transform,
                        parent_link,
                        child_link, 
                        poseMsg,
                        transformMsg,
                    visMarker);


        //生成关节角Msg
        JntToMsg(jnt_info, jointStatesMsg);


       //发送节点
        pose_pub.publish(poseMsg);
        transform_pub.publish(transformMsg);
        marker_pub.publish(visMarker);
        joint_pub.publish(jointStatesMsg);

        ros::Duration(0.1).sleep();
        ros::spinOnce();

    }

}

void SurgRobotROSP::SurgRobotRTcontrol(void)
{
    // surgTask->stopControl();
    // sleep(3);
    ui->RTcontrol->setDisabled(true);
    ui->SCcontrol->setDisabled(true);
    ui->Stop->setDisabled(false);
    surgTask->startRTControl();

}

void SurgRobotROSP::SurgRobotSCcontrol(void)
{
    ui->RTcontrol->setDisabled(true);
    ui->SCcontrol->setDisabled(true);
    ui->Stop->setDisabled(false);

    Eigen::Matrix<double,6,1> pose_marker_target;

    surgTask->findInitCuttingPoint(pose_marker_target);


    Eigen::Matrix<double,4,4> T_marker_target;
    PoseToTransform(pose_marker_target,T_marker_target);

    Eigen::Matrix<double,6,1> pose_base_marker;
    Eigen::Matrix<double,4,4> T_base_marker;
    surgTask->getRegistObject(pose_base_marker);
    PoseToTransform(pose_base_marker,T_base_marker);

    Eigen::Matrix<double,4,4> T_base_target =T_base_marker*T_marker_target;

    Eigen::Matrix<double,6,1> pose_base_target;
    TransformToPose(T_base_target,pose_base_target);


    std::cout<<"pose_base_target = "<<pose_base_target.transpose()<<std::endl;


    surgTask->startScriptControl(pose_base_target);
}

void SurgRobotROSP::SurgRobotStop(void)
{
    ui->RTcontrol->setDisabled(false);
    ui->SCcontrol->setDisabled(false);
    ui->Stop->setDisabled(true);
    surgTask->stopControl();
}




int main(int argc, char* argv[])
{
    //界面格式
    int argc1 = 0;
    char** argv1 = NULL;
    QApplication a(argc1,argv1);
    SurgRobotROSP w(NULL,argc, argv);
    w.show();

    return a.exec();


}







