#include "DianaAdmitControl.hpp"
#include "MillingTask.hpp"
#include "yaml-cpp/yaml.h"


DianaAdmitControl::DianaAdmitControl(char* ip_robot, char* serial_ee, char* serial_hh, char* ip_cam, char* path)
:DualFoSenDriver(serial_ee,serial_hh),
CameraComm(ip_cam),
DianaDriver(ip_robot)
{

    
    pftask = new MillingTask(path,6);

    instancex = new TD2Order(0.001,1000.0,0.05);
    instancey = new TD2Order(0.001,1000.0,0.05);
    // pftask = new PathFollowTask(path,4);

    // sensor_layer = new DualFoSenDriver(serial_ee,serial_hh);
    // sensor_layer = new DualFoSenDriver(serial_hh,serial_hh);
    //R_RoTohh<<0,-1,0,-1,0,0,0,0,-1;

    // Eigen::MatrixXd plannedpoints;
    // plannedpoints.resize(3,0);
    

    initFrame();
}

DianaAdmitControl::~DianaAdmitControl()
{
    
    if(getControlMode()!=T_MODE_NONE)
    {
        thread_control.detach();
    }

    delete pftask;
    delete instancex;
    delete instancey;
    //thread_control.detach();
}

int DianaAdmitControl::initFrame()
{
    theta1 = 0.0;
    //塑料安装

    Eigen::Matrix<double,4,4> T_temp;
    Eigen::Matrix<double,6,1> pose_ro_hh;
    Eigen::Matrix<double,6,1> pose_ro_ee;
    // pose_tool_hh<<0.0, 0.05, 0.0, -M_PI_2,0.0,0.0;

    // pose_tool_hh<<0.0, 0.00, -0.05, M_PI,0.0,-M_PI_2;
/***************磨钻工件********************/
    //金属件安装
    YAML::Node config_ro_hh = YAML::LoadFile("/home/agr/catkin_ws/src/diana_driver/path/configRoToHH.yaml");
    YAML::Node config_ro_ee = YAML::LoadFile("/home/agr/catkin_ws/src/diana_driver/path/configRoToEE.yaml");
    YAML::Node config_tcp_tool = YAML::LoadFile("/home/agr/catkin_ws/src/diana_driver/path/configRoToTool.yaml");
/***************磨钻工件********************/
    //金属件安装
    pose_ro_hh<<config_ro_hh["x"].as<double>(), 
                config_ro_hh["y"].as<double>(), 
                config_ro_hh["z"].as<double>(), 
                config_ro_hh["rx"].as<double>(), 
                config_ro_hh["ry"].as<double>(), 
                config_ro_hh["rz"].as<double>();

    pose_ro_ee<<config_ro_ee["x"].as<double>(), 
                config_ro_ee["y"].as<double>(), 
                config_ro_ee["z"].as<double>(), 
                config_ro_ee["rx"].as<double>(), 
                config_ro_ee["ry"].as<double>(), 
                config_ro_ee["rz"].as<double>();

    pose_tcp_tool<<config_tcp_tool["x"].as<double>(), 
                config_tcp_tool["y"].as<double>(), 
                config_tcp_tool["z"].as<double>(), 
                config_tcp_tool["rx"].as<double>(), 
                config_tcp_tool["ry"].as<double>(), 
                config_tcp_tool["rz"].as<double>();
/************END***磨钻工件********************/
/***************来复锯工件********************/
    // //金属件安装
    // pose_ro_hh<<0.147, 0.0, -0.044, -M_PI_2,0.0,-M_PI_2;
    // //确定EE坐标系
    // pose_ro_ee<<0.075, -0.040, -0.044, M_PI_2, M_PI/3.0, 0.0;
    // pose_tcp_tool<<0.075,
    //                 -0.078,
    //                 -0.073,
    //                 0.0,
    //                 0.0,
    //                 -M_PI_2;
    /************END***来复锯工件********************/


    //确定hh坐标系
    PoseRPY2Transform(pose_ro_hh,T_temp);
    WrenchTransMatrix(T_temp,A_RoTohh);

    Eigen::Matrix<double,4,4> T_RoToTool;
    PoseRPY2Transform(pose_tcp_tool,T_RoToTool);


    Eigen::Matrix<double,4,4> T_toolTohh = T_RoToTool.inverse()*T_temp;
    // Af_toolTohh.setZero();
    WrenchTransMatrix(T_toolTohh,Af_toolTohh);


    Eigen::Matrix<double,4,4> T_RoToee;
    PoseRPY2Transform(pose_ro_ee,T_RoToee);
    WrenchTransMatrix(T_RoToee,A_RoToee);
    
    std::cout<<"T_RoToTool = "<<T_RoToTool<<std::endl;
    Eigen::Matrix<double,4,4> T_toolToee = T_RoToTool.inverse()*T_RoToee;

    
    WrenchTransMatrix(T_toolToee,Af_toolToee);
  

    return 1;
}

int DianaAdmitControl::getPlanInfo(Eigen::Matrix<double, 4, 4>& plan_info)
{
    plan_info = pftask->baseFrame;
    return 1;
}

int DianaAdmitControl::startRTControlSP()
{
    //打开抱闸
    
    DianaReleaseBrake();
    sleep(3);

    //切换控制模式
    int ret = DIANAchangeControlMode(T_MODE_CART_IMPEDANCE);
    if (ret<0)
    {
        return -1;
    }

    setZeroPoint();
    SwitchRTMode(T_MODE_RT);

    //RT Thread
    thread_control = std::thread(std::mem_fn(&DianaAdmitControl::RTControlThreadSP),this);
    threadRT::setScheduling(thread_control, SCHED_RR, 80);

    return 1;

}


int DianaAdmitControl::startRTControl()
{
    //打开抱闸
    
    DianaReleaseBrake();
    sleep(3);

    //切换控制模式
    int ret = DIANAchangeControlMode(T_MODE_CART_IMPEDANCE);
    if (ret<0)
    {
        return -1;
    }

    setZeroPoint();
    SwitchRTMode(T_MODE_RT);

    //RT Thread
    thread_control = std::thread(std::mem_fn(&DianaAdmitControl::RTControlThread),this);
    threadRT::setScheduling(thread_control, SCHED_RR, 80);

    return 1;

}

int DianaAdmitControl::getObjectInfo(Eigen::Matrix<double, 7, 1>& obj_info)
{
    lock_task.lock();
    obj_info(0) = regist_inform_object(0);
    obj_info(1) = regist_inform_object(1);
    obj_info(2) = regist_inform_object(2);
    obj_info(3) = regist_inform_object(3);
    obj_info(4) = regist_inform_object(4);
    obj_info(5) = regist_inform_object(5);
    obj_info(6) = regist_inform_object(6);
    lock_task.unlock();
    return 1;
}

int DianaAdmitControl::getJntInfo(Eigen::Matrix<double, 7, 1>& jnt_info)
{
    lock_task.lock();
    jnt_info(0) = joint_state(0);
    jnt_info(1) = joint_state(1);
    jnt_info(2) = joint_state(2);
    jnt_info(3) = joint_state(3);
    jnt_info(4) = joint_state(4);
    jnt_info(5) = joint_state(5);
    jnt_info(6) = joint_state(6);
    lock_task.unlock();

    return 1;
}

int DianaAdmitControl::findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose)
{
    pftask->findInitCuttingPoint(pose);
    std::cout<<"poseccc = "<<pose<<std::endl;
    return 1;
}

int DianaAdmitControl::getPoseTCPtool(Eigen::Matrix<double,6,1>& pose_tcp_tool_l)
{
    pose_tcp_tool_l  = pose_tcp_tool;
    return 1;
}




int DianaAdmitControl::startScriptControl(Eigen::Matrix<double,6,1> target)
{
    //打开抱闸 使用脚本控制，不考虑避障 Ver1
    
    DianaReleaseBrake();
    getRegistObject(regist_inform_object);
    sleep(2);

    //切换控制模式
    int ret = DIANAchangeControlMode(T_MODE_POSITION);
    if (ret<0)
    {
        return -1;
    }

    // setZeroPoint();
    SwitchRTMode(T_MODE_SCRIPT);


    Eigen::Matrix<double,4,4> T_tcp_tool;
    PoseToTransform(pose_tcp_tool,T_tcp_tool);

    Eigen::Matrix<double,4,4> T_base_tool;
    PoseToTransform(target,T_base_tool);

    Eigen::Matrix<double,4,4> T_base_tcp = T_base_tool*T_tcp_tool.inverse();
    Eigen::Matrix<double,6,1> pose_tcp;
    TransformToPose(T_base_tcp,pose_tcp);
    Eigen::MatrixXd pose_tcp_copy = pose_tcp;


    //script Thread
    thread_control = std::thread(std::mem_fn(&DianaAdmitControl::ScriptControlThread),this,pose_tcp_copy);
    //开个线程
    threadRT::setScheduling(thread_control, SCHED_RR, 40);


}

int DianaAdmitControl::stopControl()
{
    std::cout<<"Exit to Thread"<<std::endl;
    SwitchRTMode(T_MODE_NONE);
    sleep(0.5);
    std::cout<<"Hold Break"<<std::endl;
    DianaholdBrake();
    std::cout<<"Thread Detach"<<std::endl;
    thread_control.join();
    sleep(2);
}

int DianaAdmitControl::AdmittanceControl(const Eigen::Matrix<double, 6, 1> wrench,Eigen::Matrix<double, 6, 1>& delta)
{
  
    Eigen::DiagonalMatrix<double,6> Admittance;
    //Admittance.diagonal() << 0.0,0.0,0.1,0.0,0.0,0.0;
    
    Admittance.diagonal() << 0.0,0.0,0.1,0.0,0.0,0.0;
    //Admittance.diagonal() << 0.0,0.0,0.0,0.5,0.0,0.0;
  
    delta=Admittance*Af_toolTohh*wrench;

    Eigen::Matrix<double, 6,1> pose_base_tcp;
    getCurrentTCPPose(pose_base_tcp);

    Eigen::Matrix<double,4,4> T_base_tcp,T_tcp_tool;
    PoseToTransform(pose_base_tcp,T_base_tcp);
    PoseRPY2Transform(pose_tcp_tool,T_tcp_tool);

    Eigen::Matrix<double,4,4> T_base_tool = T_base_tcp*T_tcp_tool;
    Eigen::Matrix<double,6,1> pose_base_tool;
    TransformToPose(T_base_tool,pose_base_tool);


    Eigen::Matrix<double, 6, 6> RR1;
    RR1.setZero();
    RR1.topLeftCorner(3,3) = T_base_tcp.block(0,0,3,3);
    RR1.bottomRightCorner(3,3) = T_base_tcp.block(0,0,3,3);
    // TwistTransMatrix(T_base_tcp,RR1);

    Eigen::Matrix<double,6,6> A_tcp_tool;
    A_tcp_tool.setZero();
    TwistTransMatrix(T_tcp_tool,A_tcp_tool);
    delta = RR1*A_tcp_tool*delta;


    return 1;
}
int DianaAdmitControl::AdmittanceControl(const Eigen::Matrix<double, 6, 1> wrench,
                                                        Eigen::Matrix<double, 4, 4> T_base_actual, 
                                                        Eigen::Matrix<double, 4, 4> T_base_target,
                                                        Eigen::Matrix<double, 6, 1>& delta)
{

    Eigen::DiagonalMatrix<double,6> Admittance;
    //Admittance.diagonal() << 0.0,0.0,0.1,0.0,0.0,0.0;
    
    Admittance.diagonal() << 0.0,0.0,0.1,0.0,0.0,0.0;
    //Admittance.diagonal() << 0.0,0.0,0.0,0.5,0.0,0.0;
  
    delta=Admittance*Af_toolTohh*wrench;


        

    Eigen::Matrix<double,3,3> R_diff = T_base_actual.block(0,0,3,3).transpose()*T_base_target.block(0,0,3,3);
    Eigen::Matrix<double,4,4> T_diff = T_base_actual.inverse()*T_base_target;

    Eigen::Matrix<double,3,1> r_diff, n_actual_target,n_actual_actual;
    n_actual_target = R_diff.block(0,2,3,1);
    n_actual_actual<<0.0,0.0,1.0;

    if(n_actual_target.transpose()*n_actual_actual<1e-6)
    {
        delta(0) = 0.0;
        delta(1) = 0.0;
    }
    else
    {
        double x_target = T_diff(0,3);
        double y_target = T_diff(1,3);

        double x_filtered = 0.0;
        double y_filtered = 0.0;

        double x_filteredDot = 0.0;
        double y_filteredDot = 0.0;

        instancex->update(x_target,x_filtered,x_filteredDot);
        instancey->update(y_target,y_filtered,y_filteredDot);

        delta(0) = 1.0*x_filtered;
        delta(1) = 1.0*y_filtered;


    }
    Skew_inv(R_diff,r_diff);
    // delta.setZero();

    delta.block(3,0,3,1) = r_diff*0.5;



    Eigen::Matrix<double, 6,1> pose_base_tcp;
    getCurrentTCPPose(pose_base_tcp);

    Eigen::Matrix<double,4,4> T_tcp_tool,T_base_tcp;
    PoseRPY2Transform(pose_tcp_tool,T_tcp_tool);
    PoseToTransform(pose_base_tcp,T_base_tcp);

    // Eigen::Matrix<double,4,4> T_base_tool = T_base_tcp*T_tcp_tool;
    // Eigen::Matrix<double,6,1> pose_base_tool;
    // TransformToPose(T_base_tool,pose_base_tool);


    Eigen::Matrix<double, 6, 6> RR1;
    RR1.setZero();
    RR1.topLeftCorner(3,3) = T_base_tcp.block(0,0,3,3);
    RR1.bottomRightCorner(3,3) = T_base_tcp.block(0,0,3,3);
    // TwistTransMatrix(T_base_tcp,RR1);

    Eigen::Matrix<double,6,6> A_tcp_tool;
    A_tcp_tool.setZero();
    TwistTransMatrix(T_tcp_tool,A_tcp_tool);
    delta = RR1*A_tcp_tool*delta;


    return 1;
}

int DianaAdmitControl::AdmittanceControlFSVer(const Eigen::Matrix<double, 6, 1> wrench,Eigen::Matrix<double, 6, 1>& delta)
{
    /*Time Usage*/
    // static struct timeval start;
    // static struct timeval end;
    // static int iter = 0;

    /***在该函数中获取当前TCP位置、环境力信息，人机交互力信息是该函数的输入***/


    Eigen::Matrix<double, 6,1> pose_base_tcp;
    getCurrentTCPPose(pose_base_tcp);

    Eigen::Matrix<double, 6,1> pose_base_task;
    getRegistObject(pose_base_task);


    Eigen::Matrix<double,6,1> tcp_vel;
    tcp_vel.setZero();

    Eigen::Matrix<double,6,1> wrench_ee;
    bool flag = getWrenchee(wrench_ee);


    Eigen::Matrix<double,6,1> wrench2 = Af_toolTohh * wrench;
    Eigen::Matrix<double,6,1> wrench1 = Af_toolToee * wrench_ee;

    // std::cout<<"Af_toolToee="<<Af_toolToee<<std::endl;
    // std::cout<<"w1="<<wrench1.transpose()<<std::endl;
    saveData(std::string("/home/agr/catkin_ws/wrench2.csv"), wrench2.transpose());
    saveData(std::string("/home/agr/catkin_ws/wrench_ee.csv"), wrench_ee.transpose());
    saveData(std::string("/home/agr/catkin_ws/pose_base_tcp.csv"), pose_base_tcp.transpose());

    
    Eigen::Matrix<double,4,4> T_base_tcp,T_tcp_tool;
    PoseToTransform(pose_base_tcp,T_base_tcp);
    PoseRPY2Transform(pose_tcp_tool,T_tcp_tool);

    Eigen::Matrix<double,4,4> T_base_tool = T_base_tcp*T_tcp_tool;
    Eigen::Matrix<double,6,1> pose_base_tool;
    TransformToPose(T_base_tool,pose_base_tool);


    // std::cout<<"pose_base_task = "<<pose_base_task.transpose()<<std::endl;

    delta.setZero();
 
    /*有力控制模式*/
    pftask->DynamicPlanVel(pose_base_tool,
                            pose_base_task,
                            wrench2,
                            wrench1,
                            delta,
                            theta1);
    Eigen::Matrix<double, 6, 6> RR1;
    RR1.setZero();
    RR1.topLeftCorner(3,3) = T_base_tcp.block(0,0,3,3);
    RR1.bottomRightCorner(3,3) = T_base_tcp.block(0,0,3,3);
    // TwistTransMatrix(T_base_tcp,RR1);

    Eigen::Matrix<double,6,6> A_tcp_tool;
    A_tcp_tool.setZero();
    TwistTransMatrix(T_tcp_tool,A_tcp_tool);
    delta = RR1*A_tcp_tool*delta;


    return 1;
}


void DianaAdmitControl::ScriptControlThread(Eigen::Matrix<double,6,1> target)
{
    DIANAmoveJToPose(target);
    std::cout<<"[info!!] Runing the Robot!"<<std::endl;
    DianaStop();
}


void DianaAdmitControl::RTControlThread()
{
    int rett = -1;
    Eigen::Matrix<double, 6, 1> wrench_hh_eigen;
    Eigen::Matrix<double, 6, 1> wrench_ee;

    Eigen::Matrix<double, 6, 1> pose;
    Eigen::Matrix<double, 6, 1> pose_current;
    Eigen::Matrix<double, 6, 1> delta;

    // Eigen::Matrix<double, 7, 1> regist_inform_object;

    Eigen::Matrix<double,7,1> marker_pose;

    // Eigen::Matrix<double,7,1> joint_state;

    Eigen::Matrix<double,4,4> T_temp;
    wrench_hh_eigen.setZero();
    wrench_ee.setZero();
    pose.setZero();
    pose_current.setZero();
    delta.setZero();
    regist_inform_object.setZero();
    marker_pose.setZero();
    
    // sleep(2);
    // setZeroPoint();

    while(getControlMode()==T_MODE_RT)
    {
        //getWrenchee(wrench_ee);
        //std::cout<<"EE Wrench = "<<wrench_ee.transpose()<<std::endl;
        wrench_hh_eigen.setZero();
        bool ret = getWrenchhh(wrench_hh_eigen);
        saveData(std::string("/home/agr/catkin_ws/wrench_hh_eigen.csv"), wrench_hh_eigen.transpose());

        // sensor_layer->getWrenchhh(wrench_hh_eigen);
        // // if(!ret){
            // std::cout<<"HH Wrench = "<<wrench_hh_eigen.transpose()<<std::endl;
        // }
        // std::cout<<"ret = "<<ret<<std::endl;

        getJointState(joint_state);

        getRegistObject(regist_inform_object);

        // std::cout<<"regist_inform_object = "<<regist_inform_object.transpose()<<std::endl;


        //delta<<wrench_hh_eigen[0]*0.2,wrench_hh_eigen[1]*-0.2,wrench_hh_eigen[2]*-0.2,0,0,0;
        //delta<<0.0,-0.0,0.0,0.0,0.0,0.1;
        getCurrentTCPPose(pose_current);

        AxisAnglePose2QuatPose(pose_current,marker_pose);
        // std::cout<<"pose_current1 = "<<pose_current.transpose()<<std::endl;
        // std::cout<<"marker_pose = "<<marker_pose.transpose()<<std::endl;
        


        PoseToTransform(pose_current,T_temp);
        TransformToPose(T_temp,pose_current);

        // std::cout<<"pose_current2 = "<<pose_current.transpose()<<std::endl;



        // QuatPose2EulerPoseRPY(marker_pose,pose_current);
        // std::cout<<"pose_current = "<<pose_current.transpose()<<std::endl;



        // std::cout<<"joint_state= "<<joint_state.transpose()<<std::endl;

        // std::cout<<"joint_state= "<<joint_state.transpose()<<std::endl;


        /*机械臂运动*/
        AdmittanceControlFSVer(wrench_hh_eigen, delta);
        rett = setVel(delta);
        // std::cout<<"delta= "<<delta.transpose()<<std::endl;
        // std::cout<<"Wrench = "<<wrench_hh_eigen.transpose()<<std::endl;
        if(rett<0){
             printf("\n\nFAIL\n\n");
         }


        sleep(0.05);
    }
    
    DianaStop();
}


void DianaAdmitControl::RTControlThreadSP()
{
    int rett = -1;
    Eigen::Matrix<double, 6, 1> wrench_hh_eigen;
    Eigen::Matrix<double, 6, 1> wrench_ee;

    Eigen::Matrix<double, 6, 1> pose;
    Eigen::Matrix<double, 6, 1> delta;

    // Eigen::Matrix<double, 7, 1> regist_inform_object;

    Eigen::Matrix<double,7,1> marker_pose;

    // Eigen::Matrix<double,7,1> joint_state;

    Eigen::Matrix<double,6,1> pose_base_tcp;
    // Eigen::Matrix<double,4,4> T_temp;
    Eigen::Matrix<double,4,4> T_base_tcp;
    wrench_hh_eigen.setZero();
    wrench_ee.setZero();
    pose.setZero();
    // pose_current.setZero();
    delta.setZero();
    // regist_inform_object.setZero();
    marker_pose.setZero();

    Eigen::Matrix<double,6,1> pose_marker_target;
    pose_marker_target.setZero();
    // 韩哲 随动
    Eigen::MatrixXd plannedpoints;
    std::string pathsp_str("/home/agr/catkin_ws/src/diana_driver/path/MED_SplineNDI.txt");
    importPointFromFile(pathsp_str,plannedpoints);

    std::cout<<"plannedpoints = "<<plannedpoints<<std::endl;
    pose_marker_target<<plannedpoints(0,0), plannedpoints(1,0)-0.1, plannedpoints(2,0),M_PI_2,  M_PI_2, 0.0;
    
    Eigen::Matrix<double,4,4> T_marker_target,T_tcp_tool;
    PoseRPY2Transform(pose_marker_target,T_marker_target);
    PoseToTransform(pose_tcp_tool,T_tcp_tool);

    Eigen::Matrix<double,4,4> T_base_marker;

    Eigen::Matrix<double, 6,1> regist_inform_object_ax;

    while(getControlMode()==T_MODE_RT)
    {
        //getWrenchee(wrench_ee);
        //std::cout<<"EE Wrench = "<<wrench_ee.transpose()<<std::endl;
        wrench_hh_eigen.setZero();
        bool ret = getWrenchhh(wrench_hh_eigen);
        saveData(std::string("/home/agr/catkin_ws/wrench_hh_eigen1.csv"), wrench_hh_eigen.transpose());


        getRegistObject(regist_inform_object_ax);
        PoseToTransform(regist_inform_object_ax,T_base_marker);

        Eigen::Matrix<double,4,4> T_base_target =T_base_marker*T_marker_target;



        getCurrentTCPPose(pose_base_tcp);
        PoseToTransform(pose_base_tcp,T_base_tcp);
 

        Eigen::Matrix<double,4,4> T_base_actual =T_base_tcp*T_tcp_tool;


        /*机械臂运动(深度方向上进行拖动)*/
        AdmittanceControl(wrench_hh_eigen,T_base_actual, T_base_target,delta);

        Eigen::Matrix<double,6,1> pose_base_actual;
        TransformToPose(T_base_actual,pose_base_actual);
        saveData(std::string("/home/agr/catkin_ws/pose_base_actual1.csv"), pose_base_actual.transpose());

        Eigen::Matrix<double,6,1> pose_base_target;
        TransformToPose(T_base_target,pose_base_target);
        saveData(std::string("/home/agr/catkin_ws/pose_base_target1.csv"), pose_base_target.transpose());
        saveData(std::string("/home/agr/catkin_ws/regist_inform_object_ax.csv"), regist_inform_object_ax.transpose());
        
        // std::cout<<"r_diff= "<<r_diff.transpose()<<std::endl;
        // std::cout<<"R_diff= "<<R_diff<<std::endl;


        rett = setVel(delta);
        // std::cout<<"delta= "<<delta.transpose()<<std::endl;
        // std::cout<<"Wrench = "<<wrench_hh_eigen.transpose()<<std::endl;
        if(rett<0){
             printf("\n\nFAIL\n\n");
         }


        sleep(0.05);
    }
    
    DianaStop();
}


//./devel/lib/diana_driver/CameraSen_node_test '0.0.0.0'
//./devel/lib/diana_driver/diana_driver_admittance '192.168.10.75' '/dev/ttyUSB0' '/dev/ttyUSB1' '127.0.0.1' 'src/diana_driver/path/Spline 1.txt'


int main(int argc, char* argv[])
{
    //
    char* ip_robot = argv[1];
    char* serial_ee = argv[2];
    char* serial_hh = argv[3];
    char* ip_upper = argv[4];
    char* path = argv[5];

    // DianaAdmitControl instance(argc, argv);
    DianaAdmitControl instance(ip_robot,serial_ee,serial_hh,ip_upper,path);

    /*
     RT控制方法测试
    */
//    instance.stopControl();
//    sleep(10);
    instance.startRTControl();
    sleep(10);
    instance.stopControl();

    /*
    离线控制方法测试
    */
    // Eigen::Matrix<double, 6, 1> pose_current;
    // Eigen::Matrix<double, 6, 1> pose_target;

    // instance.getCurrentTCPPose(pose_current);

    // pose_target = pose_current;
    // pose_target(1) = pose_current(1)-0.1;
    // pose_target(2) = pose_current(2)+0.1;
    // pose_target(3) = pose_current(3)+0.05;

    // std::cout<<"pose_current = "<<pose_current.transpose()<<std::endl;

    // instance.startScriptControl(pose_target);
    // sleep(2);
    // instance.stopControl();


    return 0;


}



