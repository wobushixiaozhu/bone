
#ifndef __SURROB_H
#define __SURROB_H

#include <QWidget>
#include <QString>
#include <QApplication>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "DianaAdmitControl.hpp"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace Ui {
class SurgRobotROS;
}

class SurgRobotROS : public QWidget
{
    
    Q_OBJECT

public:
    explicit SurgRobotROS(QWidget *parent=0,int argc=0,char **argv=NULL);
    ~SurgRobotROS();

private slots:
    void SurgRobotRTcontrolSP(void);
    void SurgRobotSCcontrolSP(void);
    void SurgRobotRTcontrol(void);
    void SurgRobotSCcontrol(void);
    void SurgRobotStop(void);
    

private:
    void SurgRobotVisThread(void);
    
    //单独给run开线程，死循环
    // void run();
    Ui::SurgRobotROS *ui;

    DianaAdmitControl* surgTask;
    
    ros::Publisher pose_pub;
    ros::Publisher transform_pub;
    ros::Publisher joint_pub;
    ros::Publisher marker_pub;
    bool isRunning;
    std::thread thread_vis;



    // ros::Subscriber handeyepose_sub;
    // ros::Subscriber manipose_sub;
    // ros::Publisher pose_pub;
    // ros::Publisher transform_pub;
    // ros::Publisher position_pub;
	// //Rviz 可视化
    // ros::Publisher marker_pub;

    // int num;
    // Eigen::Matrix<double,4,4> T_ee_marker;
    // Eigen::Matrix<double,4,4> T_marker_ee;
    // // Eigen::Matrix<double,4,4> T_base_ee;
    // Eigen::Matrix<double,4,4> T_camera2marker;
    // Eigen::Matrix<double,4,4> T_cc;

    // Eigen::Matrix<double,7,1> q_base_camera;

    // Eigen::Matrix<double,4,4> T_marker2base;

    // std::vector<cv::Mat> R5_marker2base;
    // std::vector<cv::Mat> t5_marker2base;
    // std::vector<cv::Mat> R5_camera2marker;
    // std::vector<cv::Mat> t5_camera2marker;

    // geometry_msgs::TransformStamped static_transformStamped;
    // // Eigen::Matrix<double,4,4> T_base2marker;
 
    // cv::Mat R_camera2marker;
    // cv::Mat t_camera2marker;
    // cv::Mat R_marker2base;
    // cv::Mat t_marker2base;
    // cv::Mat R_base2camera;
    // cv::Mat t_base2camera;
    // std::thread thread;

    

};


#endif