#ifndef __ADMITTANCECONTROL_HPP
#define __ADMITTANCECONTROL_HPP

#include "CameraComm.hpp"
#include "DualFoSenDriver.hpp"
#include "diana_driver.hpp"
#include "PathFollowTask.hpp"
#include "TD2Order.hpp"


class DianaAdmitControl:public DualFoSenDriver,public CameraComm,public DianaDriver{
    public:
        DianaAdmitControl(char* ip_robot, char* serial_ee, char* serial_hh, char* ip_cam, char* path);
        ~DianaAdmitControl();
        int startRTControl();
        int startRTControlSP();
        int startScriptControl(Eigen::Matrix<double,6,1> target);
        int stopControl();
        int getObjectInfo(Eigen::Matrix<double, 7, 1>& obj_info);
        int getJntInfo(Eigen::Matrix<double, 7, 1>& jnt_info);
        int getPlanInfo(Eigen::Matrix<double, 4, 4>& plan_info);
        int findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose);
        int getPoseTCPtool(Eigen::Matrix<double,6,1>& pose_tcp_tool_l);
        // Eigen::Matrix<double,6,1> pose_marker_target;


    private:
        // Eigen::MatrixXd plannedpoints;
        void RTControlThread();
        void RTControlThreadSP();
        int initFrame();
        void ScriptControlThread(Eigen::Matrix<double,6,1> target);
        int AdmittanceControl(const Eigen::Matrix<double, 6, 1> wrench,Eigen::Matrix<double, 6, 1>& delta);
        int AdmittanceControl(const Eigen::Matrix<double, 6, 1> wrench,
                                                        Eigen::Matrix<double, 4, 4> T_base_actual, 
                                                        Eigen::Matrix<double, 4, 4> T_base_target,
                                                        Eigen::Matrix<double, 6, 1>& delta);
        int AdmittanceControlFSVer(const Eigen::Matrix<double, 6, 1> wrench,Eigen::Matrix<double, 6, 1>& delta);
        std::thread thread_control;
        Eigen::Matrix<double, 6, 6> A_RoTohh, A_RoToee;
        Eigen::Matrix<double,6,6> Af_toolTohh, Af_toolToee;
        PathFollowTask* pftask;
        DualFoSenDriver* sensor_layer;
        double theta1;
        Eigen::Matrix<double,6,1> pose_tcp_tool;
        ros::Publisher joint_pub;
        // 
        Eigen::Matrix<double, 7, 1> regist_inform_object;
        Eigen::Matrix<double,7,1> joint_state;
        TD2Order* instancex;
        TD2Order* instancey;

        std::mutex lock_task;



};



#endif