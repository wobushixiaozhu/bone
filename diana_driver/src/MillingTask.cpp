#include "MillingTask.hpp"

//TODO:TwistTransMatrix,PoseToTransform,TwistTransMatrix2d
//TODO :DimReductionMatrix,PoseTransform2D

MillingTask::MillingTask(char* name,int _N_spline)
:PathFollowTask(name,_N_spline)
{
   
    _b = 0.002;

    speed_const = 0.00125;
    Kp<<-0.000,0.0,0.0,-10.0;
    Kd<<-0.000,0.0,0.0,-00.0;
    admittance_human_guide<<0.003,0.0,0.0,
                            0.0,0.003,0.0,
                            0.0,0.0,0.0;
    
    DimReductionMatrix<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    vl_cmd_1 = 0.0;
    forceController = new RobustForceControl();
    MotionControllerX = new ARDCMotionControl();
    MotionControllerY = new ARDCMotionControl();
    
}

MillingTask::~MillingTask()
{
    //delete cost_function;
}




int MillingTask::DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta)
{
    //动态规划，输入变量为头颅坐标系注册信息，机械臂基座到tcp的信息，tcp下的力信息； 输出为tcp下的速度信息
    /****TIME CONSUMING****/
    static struct timeval start;
    static struct timeval end;
    static int iter = 0;
    //1. 将机械臂TCP坐标系转换到Task下
    /**NEW**/
    Eigen::Matrix<double,4,4> Task_TCP_T;
    PoseTransform3D(pose_base_tcp, pose_base_task,Task_TCP_T);
    // std::cout<<"pose_base_task = "<<pose_base_task.transpose()<<std::endl;

    //2. 将Task坐标系下的机械臂TCP坐标系转换到 Task2d的TCP坐标系下
    /*******New********/
    Eigen::Matrix<double,3,1> pose_task2d_tcp;
    DimReductionTargetFrame(Task_TCP_T,pose_task2d_tcp);

    //3.FS 标架，使用未经线性化的坐标作为输入
    if(theta<0.0)
        theta = 0.0;
    else if(theta>1.0)
        theta = 1.0;
    Eigen::Matrix<double,3,1> pose_task2d_fsframe;
    FrenetSerretFrame_Plain(pose_task2d_tcp, pose_task2d_fsframe, theta);



    //4. 将task2d坐标系下的位姿转换到fsframe下
    //std::cout<<"Run to here 4"<<std::endl;
    Eigen::Matrix<double,3,1> pose_fsframe_tcp;
    PoseTransform2D(pose_task2d_fsframe,pose_task2d_tcp,pose_fsframe_tcp);


    // /********************横向平移实验*****************************/

    // saveData(std::string("/home/agr/catkin_ws/pose_base_task.csv"), pose_base_task.transpose());
    // saveData(std::string("/home/agr/catkin_ws/pose_task2d_tcp.csv"), pose_task2d_tcp.transpose());
    // saveData(std::string("/home/agr/catkin_ws/pose_task2d_fsframe.csv"), pose_task2d_fsframe.transpose());
    // saveData(std::string("/home/agr/catkin_ws/pose_fsframe_tcp.csv"), pose_fsframe_tcp.transpose());
 

    //6. 得到人机交互的前馈速度，该前馈速度作为路径上的巡航速度

    // Eigen::Matrix<double,3,1> task2d_vel;
    Eigen::Matrix<double,6,1> tool_vel;
    AdmittanceMatrix(pose_base_tcp, pose_base_task, fh, tool_vel);
    // 7. 控制器
    Eigen::Matrix<double,6,1> _u;
    //该步骤得到的速度为fs坐标系下的速度
    Eigen::Matrix<double,6,1> pose_fsframe;
    findCurrentCuttingPoint(theta, pose_fsframe);

    Eigen::Matrix<double,4,4> task2d_fsframe_T;
    PoseToTransform(pose_fsframe,task2d_fsframe_T);

    Eigen::Matrix<double,4,4> fsframe_TCP_T = task2d_fsframe_T.inverse()*baseFrame*Task_TCP_T;


    Controller(fsframe_TCP_T,pose_fsframe_tcp,tool_vel, _u);

    // 8. 深度控制器

    //9. RCM 交互控制器
    
    
    //10. 还原到高阶
    // std::cout<<"theta = "<<theta<<std::endl;

    //速度为0 保证安全
    tcp_vel.setZero();
    tcp_vel = _u;
    // tcp_vel(0) = _u(0);
    // tcp_vel(1) = _u(1);
    // tcp_vel(5) = _u(2);

    // Eigen::Matrix<double,7,1> state_task;
    // state_task<<_u,tcp_vel,theta;
    // saveData(std::string("/home/agr/catkin_ws/state_task1.csv"), state_task.transpose());

    /****TIME CONSUMING****/
    if(iter%100 == 0)
    {
        //std::cout<<"Task_TCP_T = "<<Task_TCP_T<<std::endl;
        
        //std::cout<<"pose_task2d_tcp = "<<pose_task2d_tcp.transpose()<<std::endl;
        gettimeofday(&end, NULL);
        double timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec; 
        timeuse /= 1000000;
        //printf("timeuse = %f\n",timeuse);
        gettimeofday(&start, NULL);
        iter =0;
    }
    iter++;
    /****TIME CONSUMING****/
}


//正在使用
int MillingTask::DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1> fe,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta)
{
    //动态规划，输入变量为头颅坐标系注册信息，机械臂基座到tcp的信息，tcp下的力信息； 输出为tcp下的速度信息
    /****TIME CONSUMING****/
    static struct timeval start;
    static struct timeval end;
    static int iter = 0;
    //1. 将机械臂TCP坐标系转换到Task下
    /**NEW**/
    Eigen::Matrix<double,4,4> Task_TCP_T;
    PoseTransform3D(pose_base_tcp, pose_base_task,Task_TCP_T);
    // std::cout<<"pose_base_task = "<<pose_base_task.transpose()<<std::endl;

    //2. 将Task坐标系下的机械臂TCP坐标系转换到 Task2d的TCP坐标系下
    /*******New********/
    Eigen::Matrix<double,3,1> pose_task2d_tcp;
    // Marker->tool
    DimReductionTargetFrame(Task_TCP_T,pose_task2d_tcp);

    //3.FS 标架，使用未经线性化的坐标作为输入
    if(theta<0.0)
        theta = 0.0;
    else if(theta>1.0)
        theta = 1.0;
    Eigen::Matrix<double,3,1> pose_task2d_fsframe;
    FrenetSerretFrame_Plain(pose_task2d_tcp, pose_task2d_fsframe, theta);



    //4. 将task2d坐标系下的位姿转换到fsframe下
    //std::cout<<"Run to here 4"<<std::endl;
    Eigen::Matrix<double,3,1> pose_fsframe_tcp;
    PoseTransform2D(pose_task2d_fsframe,pose_task2d_tcp,pose_fsframe_tcp);

    saveData(std::string("/home/agr/catkin_ws/pose_base_task.csv"), pose_base_task.transpose());
    saveData(std::string("/home/agr/catkin_ws/pose_task2d_tcp.csv"), pose_task2d_tcp.transpose());
    saveData(std::string("/home/agr/catkin_ws/pose_task2d_fsframe.csv"), pose_task2d_fsframe.transpose());
    saveData(std::string("/home/agr/catkin_ws/pose_fsframe_tcp.csv"), pose_fsframe_tcp.transpose());
 

    //6. 得到人机交互的前馈速度，该前馈速度作为路径上的巡航速度

    // Eigen::Matrix<double,3,1> task2d_vel;
    Eigen::Matrix<double,6,1> tool_vel, tool_vel_reg;
    AdmittanceMatrix(pose_base_tcp, pose_base_task, fh, tool_vel);
    AdmittanceMatrix(pose_base_tcp, pose_base_task, fe, tool_vel_reg);
    // 7. 控制器
    Eigen::Matrix<double,6,1> _u;
    //该步骤得到的速度为fs坐标系下的速度
    Eigen::Matrix<double,6,1> pose_fsframe;
    findCurrentCuttingPoint(theta, pose_fsframe);

    Eigen::Matrix<double,4,4> task2d_fsframe_T;
    PoseToTransform(pose_fsframe,task2d_fsframe_T);

    Eigen::Matrix<double,4,4> fsframe_TCP_T = task2d_fsframe_T.inverse()*baseFrame*Task_TCP_T;


    // Controller(fsframe_TCP_T,pose_fsframe_tcp,tool_vel, tool_vel_reg,_u);
    Controller(fsframe_TCP_T,pose_task2d_fsframe,pose_task2d_tcp,pose_fsframe_tcp,tool_vel, tool_vel_reg,_u);



    // 8. 深度控制器

    //9. RCM 交互控制器
    
    
    //10. 还原到高阶
    std::cout<<"theta = "<<theta<<std::endl;

    //速度为0 保证安全
    tcp_vel.setZero();
    tcp_vel = _u;
    // tcp_vel(0) = _u(0);
    // tcp_vel(1) = _u(1);
    // tcp_vel(5) = _u(2);

    // Eigen::Matrix<double,7,1> state_task;
    // state_task<<_u,tcp_vel,theta;
    // saveData(std::string("/home/agr/catkin_ws/state_task1.csv"), state_task.transpose());

    /****TIME CONSUMING****/
    if(iter%100 == 0)
    {
        //std::cout<<"Task_TCP_T = "<<Task_TCP_T<<std::endl;
        
        //std::cout<<"pose_task2d_tcp = "<<pose_task2d_tcp.transpose()<<std::endl;
        gettimeofday(&end, NULL);
        double timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec; 
        timeuse /= 1000000;
        //printf("timeuse = %f\n",timeuse);
        gettimeofday(&start, NULL);
        iter =0;
    }
    iter++;
    /****TIME CONSUMING****/
}

static int counter = 0;

int MillingTask::FrenetSerretFrame_Plain(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta)
{
    //1. 给定y 
    Eigen::Matrix<double,2,1> yy(y(0),y(1));
    
    double Jo = 0.0;
    double Jaco = 0.0;
    double alpha = 0.1;
    double iter_num = 50;
    double threshold_stop = 1e-3;
    double delta_theta =0.0;
    double theta_last = theta;

    int k =0;

    int s = 0;
    double temp = 0.0;
    int min_s = 0;
    double min_Jc = 1e6;
    double Jc = 0.0;
    for(s = 0;s<=1000;s++)
    {
        temp = s*0.001;
        Jc = J(temp,yy);
        if(Jc-min_Jc<0.0 && std::abs(theta-s*0.001)<0.1)
        {
            min_s = s;
            min_Jc = Jc;
        }
    }

    // 判断是否连续，如果是首尾相接曲线不能使用全局搜索

    theta = min_s*0.001;

    if(theta<theta_last+0.003)
    {
        theta = theta_last;
    }

    // if((theta>theta_last+0.005 || theta< theta_last-0.005)&&counter<100)
    // {
    //     theta = theta_last;
    //     counter = counter+1;
    // }
    // else
    // {
    //     counter = 0;
    // }

    


    FromThetaToAgent(theta,pose);
    // theta_last = theta;

    return 1;


    
}

int MillingTask::findCurrentCuttingPoint(double theta, Eigen::Matrix<double,6,1>& pose_task2d_a)
{
    Eigen::Matrix<double,3,1> y;
    FromThetaToAgent(theta,y);


    // Eigen::Matrix<double,6,1> pose_task2d_a;
    pose_task2d_a.setZero();
    pose_task2d_a(0) = y(0);
    pose_task2d_a(1) = y(1);
    pose_task2d_a(5) = y(2);

    // Eigen::Matrix<double,4,4> T_task3d_cutP;
    // PoseToTransform(pose_task3d_cutP,T_task3d_cutP);


    // Eigen::Matrix<double,4,4> T_marker_cutP = baseFrame.inverse()*T_task3d_cutP;

    // Eigen::Matrix<double,6,1> p;
    // TransformToPose(T_marker_cutP,p);

    // pose = p;
    return 1;

}

// int MillingTask::FromThetaToAgent(double theta, Eigen::Matrix<double,3,1>& y)
// {
//     assert((theta>=0)&&(theta<=1));
//     Eigen::Matrix<double,2,1> y1;
//     sigma(theta,y1);

    
//     Eigen::Matrix<double,2,1> yDot;
//     sigmaDot(theta, yDot);

//     Eigen::Matrix<double,3,1> xx;
//     xx.setZero();
//     xx.block(0,0,2,1) = 1/(yDot.norm())*yDot;

//     Eigen::Matrix<double,3,1> xx_base;
//     xx_base<<1,0,0;

//     Eigen::Map<Eigen::Vector3d> _xx = Eigen::Vector3d::Map(xx.data(),xx.size());
//     Eigen::Map<Eigen::Vector3d> _xx_base = Eigen::Vector3d::Map(xx_base.data(),xx_base.size());

//     double _xx_dot = _xx.dot(_xx_base);
//     Eigen::VectorXd _xx_cross= _xx_base.cross(_xx);
//     double thetad = acos(_xx_dot);
//     if(_xx_cross[2]<0)
//     {
//         thetad = 2*M_PI-thetad;
//     }

//     y.block(0,0,2,1) = y1;
//     y(2) = thetad; 

//     return 1;

// }


int MillingTask::AdmittanceMatrix(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,
                                        Eigen::Matrix<double,6,1> twist, Eigen::Matrix<double,6,1>& delta)
{

    // Eigen::Matrix<double,6,1> delta;
    Eigen::DiagonalMatrix<double,6> Admittance;
    Admittance.diagonal() << 0.02,0.02,0.02,0.2,0.2,0.0;   

    delta.setZero();
    // 该过程将力（机器人TOOL）转换到了速度（机器人 TOOL）
    delta=Admittance*twist;


    // Eigen::Matrix<double,3,1> vel_temp;
    // vel_temp.setZero();
    // vel_temp(0)=twist(0);
    // vel_temp(1)=twist(1);

    // 该过程将平面力（机器人TOOL）转换到了平面速度（机器人 TOOL）

    // vel = admittance_human_guide*vel_temp;

    // 需要将该平面速度从把手（机器人坐标系）下转移任务（轨迹坐标系）下

    //std::cout<<"twist_task2d = "<<twist_task2d.transpose()<<std::endl;
    // vel(0) =0.0;
    // vel(1) =0.0;
    // vel(2) =0.0;
    
    return 1;
}
// int MillingTask::RCMAdmittance(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,
//                                         Eigen::Matrix<double,6,1> twist, Eigen::Matrix<double,2,1>& vel)

// {
//     // 拖动
//     Eigen::Matrix<double,6,1> delta;
//     Eigen::DiagonalMatrix<double,6> Admittance;
//     Admittance.diagonal() << 0.05,0.05,0.05,0.2,0.2,0.2;   

//     delta.setZero();
//     delta=RR1*Admittance*A_RoTohh*twist;

//     vel(0) = delta(3);
//     vel(0) = delta(4);

//     return 1;
// }

/*当前使用的Controller*/

int MillingTask::Controller(Eigen::Matrix<double,4,4> Task_TCP_T,
                        Eigen::Matrix<double,3,1> pose_task2d_fsframe,
                        Eigen::Matrix<double,3,1> pose_task2d_tool,
                            Eigen::Matrix<double,3,1> pose_fsframe_tool,  
                            Eigen::Matrix<double,6,1> tool_tool_vel_human_guide, 
                            Eigen::Matrix<double,6,1> tool_tool_vel_environmentRes, 
                            Eigen::Matrix<double,6,1>& v_syn)
{
        //此处task坐标系为fs标价坐标系
    //1. 求得了平面坐标系fs标架下的机器人位置误差
    Eigen::Matrix<double,2,1> p;
    p<< pose_fsframe_tool(0),pose_fsframe_tool(1);


    //2. 得到转换矩阵，用于将tool坐标系下的速度转换为fs标架下(分解速度)

    //这里的TCP就是Tool，之前笔误了
    Eigen::Matrix<double,6,6> A_Task_TCP;
    // TwistTransMatrix(Task_TCP_T,A_Task_TCP);
    A_Task_TCP.setZero();
    A_Task_TCP.block(0,0,3,3) = Task_TCP_T.block(0,0,3,3);
    A_Task_TCP.block(3,3,3,3) = Task_TCP_T.block(0,0,3,3);

    Eigen::Matrix<double,6,1> v_temp = A_Task_TCP*tool_tool_vel_human_guide;
    Eigen::Matrix<double,6,1> v_temp2 = A_Task_TCP*tool_tool_vel_environmentRes;


    Eigen::DiagonalMatrix<double,2> S;
    S.diagonal() << 10.0,00.0;

    //3. 横向自动控制和切向拖动控制，输出vw
    Eigen::Matrix<double,2,1> vw;

    double r_signal = pose_task2d_fsframe(1);
    double y_signal = pose_task2d_tool(1);

    double u_trackx = 0.0;//-0.0*pose_fsframe_tool(0);//MotionControllerX->Update(0.0,pose_fsframe_tool(0));
    double u_tracky = MotionControllerY->Update(0.0,pose_fsframe_tool(1));//-10.0*pose_fsframe_tool(1);//

    Eigen::Matrix<double,2,1> vw_temp;
    vw_temp<<u_trackx,u_tracky;

    Eigen::Matrix<double,4,4> TCP_TASK_T = Task_TCP_T.inverse();
    // double u_track = -sin(pose_task2d_tool(2))*cos(pose_task2d_fsframe(2))*u_trackx+cos(pose_task2d_tool(2))*sin(pose_task2d_fsframe(2))*u_tracky;
    // std::cout<<"pose_fsframe_tool(2) = "<<pose_fsframe_tool(2)<<std::endl;
    vw = Kp*p+Kd*(p-p_t_1) + S*v_temp.block(0,0,2,1);
    p_t_2 = p_t_1;
    p_t_1 = p;

    // vw(0) = speed_const;
    v_syn.setZero();
    // v_syn(0) = cos(pose_fsframe_tool(2))*(u_trackx+speed_const) + sin(pose_fsframe_tool(2))*u_tracky;
    // v_syn(1) = -sin(pose_fsframe_tool(2))*(u_trackx+speed_const) + cos(pose_fsframe_tool(2))*u_tracky;//vw(1); //这个-1.0原理未明，但是不是负数将不收敛
    

    if(v_temp2(2)*50.0>0.2)
    {
        v_syn(0) = speed_const*(1-std::exp(-500.0*std::abs(v_temp2(2))));
    }
    else
    {
        v_syn(0) = speed_const;
    }
    v_syn(1) = u_tracky;//vw(1); //这个-1.0原理未明，但是不是负数将不收敛
    


    // for(int i=0;i<2;i++)
    // {
    //     if(vw(i) > 0.012)
    //     {
    //         vw(i) = 0.012; 
    //     }
    //     else if(vw(i) < -0.012)
    //     {
    //         vw(i) = -0.012; 
    //     }
    // }


    //4.  深度控制


    //合成坐标系：Tsyn 其姿态与TOOL一致，但位置与task一致
    // Eigen::Matrix<double,6,1> v_syn; 该向量的基坐标系为tool坐标系
    
    // TODO： 深度上的力控制
    double tempcc = forceController->Update(v_temp(2), -v_temp2(2));
    // v_syn(2) = tempcc;//v_temp(2)+v_temp2(2);
    v_syn(2) = v_temp(2)+v_temp2(2);

    for(int i=0;i<2;i++)
    {
        if(v_syn(i) > 0.022)
        {
            v_syn(i) = 0.022; 
        }
        else if(v_syn(i) < -0.022)
        {
            v_syn(i) = -0.022; 
        }
    }

    Eigen::Matrix<double,2,1> v_syncompare;
    v_syncompare<<v_syn(1), tempcc;

    saveData(std::string("/home/agr/catkin_ws/v_syncompare.csv"), v_syncompare.transpose());

    // v_syn(2) = 0.0;
    saveData(std::string("/home/agr/catkin_ws/v_syn.csv"), v_syn.transpose());

    v_syn = A_Task_TCP.inverse() * v_syn;

    v_syn(3) = tool_tool_vel_human_guide(3);
    v_syn(4) = tool_tool_vel_human_guide(4);
    v_syn(5) = 0.0;

    //TODO 控制量u到tcp上的速度
    // u.setZero();
    // 0 方向：人机交互； 1 方向：横向控制（路径控制）
    // saveData(std::string("/home/agr/catkin_ws/pose_task_tcp.csv"), pose_record.transpose());
    saveData(std::string("/home/agr/catkin_ws/v_temp.csv"), v_temp.transpose());
    saveData(std::string("/home/agr/catkin_ws/v_temp2.csv"), v_temp2.transpose());
    saveData(std::string("/home/agr/catkin_ws/vw_temp.csv"), vw_temp.transpose());

    return 1;

}

int MillingTask::Controller(Eigen::Matrix<double,4,4> Task_TCP_T,
                            Eigen::Matrix<double,3,1> pose_fsframe_tool,  
                            Eigen::Matrix<double,6,1> tool_tool_vel_human_guide, 
                            Eigen::Matrix<double,6,1> tool_tool_vel_environmentRes, 
                            Eigen::Matrix<double,6,1>& v_syn)
{
    //此处task坐标系为fs标价坐标系
    //1. 求得了平面坐标系fs标架下的机器人位置误差
    Eigen::Matrix<double,2,1> p;
    p<< pose_fsframe_tool(0),pose_fsframe_tool(1);


    //2. 得到转换矩阵，用于将tool坐标系下的速度转换为fs标架下(分解速度)
    // Eigen::Matrix<double,4,4> T_task_tool;
    // PoseToTransform(pose_task_tool,T_task_tool);
    //这里的TCP就是Tool，之前笔误了
    Eigen::Matrix<double,6,6> A_Task_TCP;
    TwistTransMatrix(Task_TCP_T,A_Task_TCP);

    Eigen::Matrix<double,6,1> pose_record;
    TransformToPose(Task_TCP_T,pose_record);

    

    Eigen::Matrix<double,6,1> v_temp = A_Task_TCP*tool_tool_vel_human_guide;
    Eigen::Matrix<double,6,1> v_temp2 = A_Task_TCP*tool_tool_vel_environmentRes;
    // std::cout<<"pose_fsframe_tool = "<<pose_fsframe_tool.transpose()<<std::endl;
    // std::cout<<"tool_tool_vel_human_guide = "<<tool_tool_vel_human_guide.transpose()<<std::endl;

    Eigen::DiagonalMatrix<double,2> S;
    S.diagonal() << 10.0,00.0;



    //3. 横向自动控制和切向拖动控制，输出vw
    Eigen::Matrix<double,2,1> vw;
    // if(v_temp(0)>0.0004||v_temp(0)<-0.0004)
    // {
    //     vw = Kp*p+Kd*(p-p_t_1) + S*v_temp.block(0,0,2,1);
    // } 
    // else
    // {
    //     vw.setZero();
    // }
    vw = Kp*p+Kd*(p-p_t_1) + S*v_temp.block(0,0,2,1);
    p_t_2 = p_t_1;
    p_t_1 = p;

    vw(0) = speed_const;
    
    // std::cout<<"vw = "<<vw.transpose()<<std::endl;
    // saveData(std::string("/home/agr/catkin_ws/vw.csv"), vw.transpose());

    // for(int i=0;i<2;i++)
    // {
    //     if(vw(i) > 0.0012)
    //     {
    //         vw(i) = 0.0012; 
    //     }
    //     else if(vw(i) < -0.0012)
    //     {
    //         vw(i) = -0.0012; 
    //     }
    // }


    for(int i=0;i<2;i++)
    {
        if(vw(i) > 0.012)
        {
            vw(i) = 0.012; 
        }
        else if(vw(i) < -0.012)
        {
            vw(i) = -0.012; 
        }
    }


    //4.  深度控制


    //合成坐标系：Tsyn 其姿态与TOOL一致，但位置与task一致
    // Eigen::Matrix<double,6,1> v_syn; 该向量的基坐标系为tool坐标系
    v_syn.setZero();
    
    // TODO： 深度上的力控制
    v_syn(0) = vw(0);
    v_syn(1) = vw(1);
    double tempcc = forceController->Update(v_temp(2)+v_temp2(2));
    v_syn(2) = tempcc;//v_temp(2)+v_temp2(2);

    Eigen::Matrix<double,2,1> v_syncompare;
    v_syncompare<<v_syn(2), tempcc;

    saveData(std::string("/home/agr/catkin_ws/v_syncompare.csv"), v_syncompare.transpose());

    // v_syn(2) = 0.0;

    v_syn = A_Task_TCP.inverse() * v_syn;

    // v_syn(3) = tool_tool_vel_human_guide(3);
    // v_syn(4) = tool_tool_vel_human_guide(4);
    v_syn(5) = 0.0;

    //TODO 控制量u到tcp上的速度
    // u.setZero();
    // 0 方向：人机交互； 1 方向：横向控制（路径控制）
    saveData(std::string("/home/agr/catkin_ws/pose_task_tcp.csv"), pose_record.transpose());
    saveData(std::string("/home/agr/catkin_ws/v_temp.csv"), v_temp.transpose());
    saveData(std::string("/home/agr/catkin_ws/v_temp2.csv"), v_temp2.transpose());
    saveData(std::string("/home/agr/catkin_ws/v_syn.csv"), v_syn.transpose());

    return 1;
}



int MillingTask::Controller(Eigen::Matrix<double,4,4> Task_TCP_T,
                            Eigen::Matrix<double,3,1> pose_fsframe_tool,  
                            Eigen::Matrix<double,6,1> tool_tool_vel_human_guide, 
                            Eigen::Matrix<double,6,1>& v_syn)
{
    //此处task坐标系为fs标价坐标系
    //1. 求得了平面坐标系fs标架下的机器人位置误差
    Eigen::Matrix<double,2,1> p;
    p<< pose_fsframe_tool(0),pose_fsframe_tool(1);


    //2. 得到转换矩阵，用于将tool坐标系下的速度转换为fs标架下(分解速度)
    // Eigen::Matrix<double,4,4> T_task_tool;
    // PoseToTransform(pose_task_tool,T_task_tool);
    //这里的TCP就是Tool，之前笔误了
    Eigen::Matrix<double,6,6> A_Task_TCP;
    TwistTransMatrix(Task_TCP_T,A_Task_TCP);

    Eigen::Matrix<double,6,1> pose_record;
    TransformToPose(Task_TCP_T,pose_record);

    

    Eigen::Matrix<double,6,1> v_temp = A_Task_TCP*tool_tool_vel_human_guide;
    std::cout<<"v_Decomp = "<<v_temp.transpose()<<std::endl;
    std::cout<<"tool_tool_vel_human_guide = "<<tool_tool_vel_human_guide.transpose()<<std::endl;

    Eigen::DiagonalMatrix<double,2> S;
    S.diagonal() << 10.0,00.0;



    //3. 横向自动控制和切向拖动控制，输出vw
    Eigen::Matrix<double,2,1> vw;
    // if(v_temp(0)>0.0004||v_temp(0)<-0.0004)
    // {
    //     vw = Kp*p+Kd*(p-p_t_1) + S*v_temp.block(0,0,2,1);
    // } 
    // else
    // {
    //     vw.setZero();
    // }
    vw = Kp*p+Kd*(p-p_t_1) + S*v_temp.block(0,0,2,1);
    p_t_2 = p_t_1;
    p_t_1 = p;

    vw(0) = speed_const;
    
    std::cout<<"vw = "<<vw.transpose()<<std::endl;

    /***************************横向控制器输出**************************************/
    // saveData(std::string("/home/agr/catkin_ws/pose_task_tcp.csv"), pose_record.transpose());
    // saveData(std::string("/home/agr/catkin_ws/vw.csv"), vw.transpose());

    for(int i=0;i<2;i++)
    {
        if(vw(i) > 0.0012)
        {
            vw(i) = 0.0012; 
        }
        else if(vw(i) < -0.0012)
        {
            vw(i) = -0.0012; 
        }
    }

    //4.  深度控制


    //合成坐标系：Tsyn 其姿态与TOOL一致，但位置与task一致
    // Eigen::Matrix<double,6,1> v_syn; 该向量的基坐标系为tool坐标系
    v_syn.setZero();
    
    // v_syn(0) = speed_const;
    v_syn(0) = vw(0);
    v_syn(1) = vw(1);
    v_syn(2) = 0.0;//v_temp(2);

    v_syn = A_Task_TCP.inverse() * v_syn;

    v_syn(3) = tool_tool_vel_human_guide(3);
    v_syn(4) = tool_tool_vel_human_guide(4);
    v_syn(5) = 0.0;

    //TODO 控制量u到tcp上的速度
    // u.setZero();
    // 0 方向：人机交互； 1 方向：横向控制（路径控制）

    return 1;
}




int main(int argc, char* argv[])
{
    char* path = argv[1];
    char* theta = argv[2];
    PathFollowTask instance(path,2);

    Eigen::Matrix<double,2,1> y;

    double th = std::stod(theta);

    instance.sigma(th,y);
    std::cout<<"y2D_Task = "<<instance.y2D_Task<<std::endl;
    std::cout<<"y = "<<y<<std::endl;

    Eigen::Matrix<double,6,1> pose_base_tcp;
    Eigen::Matrix<double,6,1> pose_base_task; 
    Eigen::Matrix<double,6,1> fh;
    Eigen::Matrix<double,6,1> tcp_vel;

    pose_base_tcp.setZero();
    pose_base_task.setZero();
    fh.setZero();
    tcp_vel.setZero();

    double theta1;

    // instance.DynamicPlanVel(pose_base_tcp,pose_base_task,fh,tcp_vel,theta1);

    std::cout<<"theta1 = "<<theta1<<std::endl;

    Eigen::Matrix<double,3,1> y3;
    y3<<0.000382948,0.0000462554,M_PI/4*5;
    //y3<<0.0382948,0.00462554,M_PI/4*5;

    Eigen::Matrix<double,3,1> pose3;
    double theta2 = 0.0;







    return 0;
    
}





