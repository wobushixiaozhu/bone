#include "DianaAdmitControlPlaneCut.hpp"



DianaAdmitControlPlaneCut::DianaAdmitControlPlaneCut(char* ip_robot, char* serial_ee, char* serial_hh, char* ip_cam, char* path)
:DualFoSenDriver(serial_ee,serial_hh),
CameraComm(ip_cam),
DianaDriver(ip_robot)
{
    pftaskl = new PlaneCuttingTask(path);
    //R_RoTohh<<0,-1,0,-1,0,0,0,0,-1;

    initFrame();
}

DianaAdmitControlPlaneCut::~DianaAdmitControlPlaneCut()
{
    
    if(getControlMode()!=T_MODE_NONE)
    {
        thread_control.detach();
    }

    delete pftaskl;
    //thread_control.detach();
}

int DianaAdmitControlPlaneCut::startRTControl()
{
    //打开抱闸
    
    DianaReleaseBrake();
    sleep(2);

    //切换控制模式
    int ret = DIANAchangeControlMode(T_MODE_CART_IMPEDANCE);
    if (ret<0)
    {
        return -1;
    }

    setZeroPoint();
    SwitchRTMode(T_MODE_RT);

    //RT Thread
    thread_control = std::thread(std::mem_fn(&DianaAdmitControlPlaneCut::RTControlThread),this);
    threadRT::setScheduling(thread_control, SCHED_RR, 80);

    return 1;

}

int DianaAdmitControlPlaneCut::getPlanInfo(Eigen::Matrix<double, 4, 4>& plan_info)
{
    plan_info = pftaskl->baseFrame;
    return 1;
}


int DianaAdmitControlPlaneCut::findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose)
{
    pftaskl->findInitCuttingPoint(pose);
    std::cout<<"poseccc = "<<pose<<std::endl;
    return 1;
}


int DianaAdmitControlPlaneCut::initFrame()
{
    theta1 = 0.0;
    //塑料安装
    // Eigen::Matrix<double, 3, 3> R_RoTohh;
    // Eigen::Matrix<double, 3, 1> t_RoTohh;
    // double L = 169.40*0.001;
    // R_RoTohh<<-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0;
    // t_RoTohh<<0,0,-L;
    // A_RoTohh.setZero();
    // A_RoTohh.topLeftCorner(3,3) = R_RoTohh;
    // A_RoTohh.bottomRightCorner(3,3) = R_RoTohh;
    // A_RoTohh.bottomLeftCorner(3,3) = Skew_c(t_RoTohh)*R_RoTohh;

    //金属件安装
    Eigen::Matrix<double,4,4> T_temp;
    Eigen::Matrix<double,6,1> pose_ro_hh;
    // pose_tool_hh<<0.0, 0.05, 0.0, -M_PI_2,0.0,0.0;

    // pose_tool_hh<<0.0, 0.00, -0.05, M_PI,0.0,-M_PI_2;

    pose_ro_hh<<0.147, 0.0, -0.044, -M_PI_2,0.0,-M_PI_2;


    PoseRPY2Transform(pose_ro_hh,T_temp);
    WrenchTransMatrix(T_temp,A_RoTohh);


    pose_tcp_tool<<0.075,
                    -0.078,
                    -0.073,
                    0.0,
                    0.0,
                    -M_PI_2;

    Eigen::Matrix<double,4,4> T_RoToTool;
    Af_toolTohh.setZero();
    T_RoToTool.setZero();
    PoseRPY2Transform(pose_tcp_tool,T_RoToTool);
    Eigen::Matrix<double,4,4> T_toolTohh = T_RoToTool.inverse()*T_temp;
    WrenchTransMatrix(T_toolTohh,Af_toolTohh);

    return 1;
}

int DianaAdmitControlPlaneCut::AdmittanceControlFSVer(const Eigen::Matrix<double, 6, 1> wrench,Eigen::Matrix<double, 6, 1>& delta)
{
    /*Time Usage*/
    // static struct timeval start;
    // static struct timeval end;
    // static int iter = 0;

    /******/


    Eigen::Matrix<double, 6,1> pose_base_tcp;
    Eigen::Matrix<double, 6,1> pose_base_task;

    getCurrentTCPPose(pose_base_tcp);
    //std::cout<<"[1]pose_base_tcp = "<<pose_base_tcp.transpose()<<std::endl;
    getRegistObject(pose_base_task);
    // pose_base_task.setZero();
    // pose_base_task(3) = 1.0;
    //std::cout<<"[2]pose_base_task = "<<pose_base_task.transpose()<<std::endl;



    Eigen::Matrix<double,6,1> tcp_vel;

    tcp_vel.setZero();

    //theta1;

    // Eigen::Matrix<double,6,1> wrench1 = A_RoTohh*wrench;
     Eigen::Matrix<double,6,1> wrench2 = Af_toolTohh*wrench;

    

    // std::cout<<"wrench = "<<wrench.transpose()<<std::endl;
    // std::cout<<"wrench1 = "<<wrench1.transpose()<<std::endl;



    Eigen::Matrix<double,6,1> pose_base_tool;

    //PoseTransform3D(pose_base_tcp,pose_tcp_tool,pose_base_tool);
    Eigen::Matrix<double,4,4> T_base_tcp,T_tcp_tool;
    PoseToTransform(pose_base_tcp,T_base_tcp);
    PoseRPY2Transform(pose_tcp_tool,T_tcp_tool);

    Eigen::Matrix<double,4,4> T_base_tool = T_base_tcp*T_tcp_tool;
    TransformToPose(T_base_tool,pose_base_tool);

    // std::cout<<"pose_base_tcp = "<<pose_base_tcp.transpose()<<std::endl;
    // std::cout<<"pose_base_tool = "<<pose_base_tool.transpose()<<std::endl;

    // std::cout<<"pose_base_task = "<<pose_base_task.transpose()<<std::endl;

    pftaskl->DynamicPlanVel(pose_base_tool,
                            pose_base_task,
                            wrench2,
                            delta,
                            theta1);

    // std::cout<<"delta = "<<delta.transpose()<<std::endl;
    // std::cout<<"theta1 = "<<theta1<<std::endl;

    
    Eigen::Matrix<double,6,6> A_tcp_tool;
    A_tcp_tool.setZero();
    TwistTransMatrix(T_tcp_tool,A_tcp_tool);

    // Eigen::Matrix<double, 4, 4> Rotation_tcp;
    // Rotation_tcp.setZero();
    // getPoseBsToTCP(Rotation_tcp);

    // Eigen::Matrix<double, 3, 3> R = Rotation_tcp.topLeftCorner(3,3);


    // Eigen::Matrix<double, 6, 6> RR;
    // RR.setZero();
    // RR.topLeftCorner(3,3) = T_base_tool.block(0,0,3,3);
    // RR.bottomRightCorner(3,3) = T_base_tool.block(0,0,3,3);


    Eigen::Matrix<double, 6, 6> RR1;
    // Eigen::Matrix<double, 4, 4> T_base_tool;
    // PoseToTransform(pose_base_tool,T_base_tool);
    RR1.setZero();
    RR1.topLeftCorner(3,3) = T_base_tcp.block(0,0,3,3);
    RR1.bottomRightCorner(3,3) = T_base_tcp.block(0,0,3,3);


    //TwistTransMatrix(T_base_tool,RR1);

    delta = RR1*A_tcp_tool*delta;


 


    return 1;
}



int DianaAdmitControlPlaneCut::getObjectInfo(Eigen::Matrix<double, 7, 1>& obj_info)
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

int DianaAdmitControlPlaneCut::getJntInfo(Eigen::Matrix<double, 7, 1>& jnt_info)
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






int DianaAdmitControlPlaneCut::startScriptControl(Eigen::Matrix<double,6,1> target)
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

    setZeroPoint();
    SwitchRTMode(T_MODE_SCRIPT);


    Eigen::Matrix<double,4,4> T_tcp_tool;
    PoseToTransform(pose_tcp_tool,T_tcp_tool);

    Eigen::Matrix<double,4,4> T_base_tool;
    PoseToTransform(target,T_base_tool);

    Eigen::Matrix<double,4,4> T_base_tcp = T_base_tool*T_tcp_tool.inverse();
    Eigen::Matrix<double,6,1> pose_tcp;
    TransformToPose(T_base_tcp,pose_tcp);

    


    //script Thread
    thread_control = std::thread(std::mem_fn(&DianaAdmitControlPlaneCut::ScriptControlThread),this,pose_tcp);
    //开个线程
    threadRT::setScheduling(thread_control, SCHED_RR, 40);


}

int DianaAdmitControlPlaneCut::stopControl()
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



void DianaAdmitControlPlaneCut::ScriptControlThread(Eigen::Matrix<double,6,1> target)
{
    DIANAmoveJToPose(target);
    std::cout<<"[info!!] Runing the Robot!"<<std::endl;
    DianaStop();
}


void DianaAdmitControlPlaneCut::RTControlThread()
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

    while(getControlMode()==T_MODE_RT)
    {
        //getWrenchee(wrench_ee);
        //std::cout<<"EE Wrench = "<<wrench_ee.transpose()<<std::endl;
        getWrenchhh(wrench_hh_eigen);

        getJointState(joint_state);

        getRegistObject(regist_inform_object);

        // std::cout<<"regist_inform_object = "<<regist_inform_object.transpose()<<std::endl;
        std::cout<<"HH Wrench = "<<wrench_hh_eigen.transpose()<<std::endl;


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


        //手眼标定的原因要注释掉这一部分
        rett = setVel(delta);
        // std::cout<<"delta= "<<delta.transpose()<<std::endl;
        if(rett<0){
             printf("\n\nFAIL\n\n");
         }


        sleep(0.05);
    }
    
    DianaStop();
}


