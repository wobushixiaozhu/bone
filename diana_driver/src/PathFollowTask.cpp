#include "PathFollowTask.hpp"

//TODO:TwistTransMatrix,PoseToTransform,TwistTransMatrix2d
//TODO :DimReductionMatrix,PoseTransform2D

PathFollowTask::PathFollowTask(char* name,int _N_spline)
:Spline2DCurve(name,_N_spline)
{
    google::InitGoogleLogging("./data/1.txt");
    

    _b = 0.002;
    //ptrPFT = this;
    options.minimizer_progress_to_stdout = true;
    //options.logging_type = ceres::SILENT;
    //options.max_num_iterations = 10;
    //options.max_solver_time_in_seconds = 1e-2;
    options.line_search_direction_type = ceres::BFGS;
    //options.num_threads = 2;
    options.min_line_search_step_size = 1e-8;
    options.linear_solver_type = ceres::DENSE_QR;
    options.update_state_every_iteration = true;
    options.line_search_sufficient_function_decrease = 1e-8;
    options.minimizer_type = ceres::LINE_SEARCH;
    speed_const = 0.00045;
    //options.min_line_search_step_contraction = 0.1;
    //options.check_gradients =false;
    // options.function_tolerance = 1e-4;
    // options.initial_trust_region_radius = 0.3;
    // options.gradient_tolerance = 5e-3;
    // options.parameter_tolerance = 1e-3;
    // options.min_trust_region_radius = 1e-6;


    Kp<<-0.000,0.0,0.0,-0.4;
    Kd<<-0.000,0.0,0.0,-0.1;
    admittance_human_guide<<0.003,0.0,0.0,
                            0.0,0.003,0.0,
                            0.0,0.0,0.0;
    
    DimReductionMatrix<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    
}

PathFollowTask::~PathFollowTask()
{
    //delete cost_function;
}

int PathFollowTask::DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta)
{
    //动态规划，输入变量为头颅坐标系注册信息，机械臂基座到tcp的信息，tcp下的力信息； 输出为tcp下的速度信息
    //1. 将机械臂TCP坐标系转换到Task下
    //std::cout<<"Run to here 1"<<std::endl;

    /****TIME CONSUMING****/
    static struct timeval start;
    static struct timeval end;
    static int iter = 0;

    /***********************/

    // Eigen::Matrix<double,6,1> Task_TCP;
    // PoseTransform3D(pose_base_tcp, pose_base_task,Task_TCP);

    /**NEW**/
    Eigen::Matrix<double,4,4> Task_TCP_T;
    PoseTransform3D(pose_base_tcp, pose_base_task,Task_TCP_T);
    // std::cout<<"pose_base_task = "<<pose_base_task.transpose()<<std::endl;

    //2. 将Task坐标系下的机械臂TCP坐标系转换到 Task2d的TCP坐标系下
    //std::cout<<"Run to here 2"<<std::endl;
    // Eigen::Matrix<double,3,1> pose_task2d_tcp;
    // DimReductionTargetFrame(Task_TCP,pose_task2d_tcp);

    /*******New********/
    Eigen::Matrix<double,3,1> pose_task2d_tcp;
    DimReductionTargetFrame(Task_TCP_T,pose_task2d_tcp);

    //3.FS 标架，使用未经线性化的坐标作为输入
    //std::cout<<"Run to here 3"<<std::endl;
    if(theta<0.0)
        theta = 0.0;
    else if(theta>1.0)
        theta = 1.0;
    Eigen::Matrix<double,3,1> pose_task2d_fsframe;
    FrenetSerretFrame_Plain(pose_task2d_tcp, pose_task2d_fsframe, theta);
    saveData(std::string("/home/agr/catkin_ws/pose_task2d_tcp.csv"), pose_task2d_tcp.transpose());
    saveData(std::string("/home/agr/catkin_ws/pose_task2d_fsframe.csv"), pose_task2d_fsframe.transpose());

    // if((pose_task2d_fsframe(0)>0.035)&&(pose_task2d_fsframe(1)>0.0024))
    // {
    //     //saveData(std::string("theta.csv"), theta);
    //     saveData(std::string("theta.csv"), pose_task2d_fsframe);
    // }


    //4. 将task2d坐标系下的位姿转换到fsframe下
    //std::cout<<"Run to here 4"<<std::endl;
    Eigen::Matrix<double,3,1> pose_fsframe_tcp;
    PoseTransform2D(pose_task2d_fsframe,pose_task2d_tcp,pose_fsframe_tcp);

    saveData(std::string("/home/agr/catkin_ws/pose_fsframe_tcp.csv"), pose_fsframe_tcp.transpose());
    //5. 基于独轮车模型的线性化
    //std::cout<<"Run to here 5"<<std::endl;
    Eigen::Matrix<double,3,1> pose_fsframe_tcp_l;
    // std::cout<<"pose_task2d_fsframe = "<<pose_task2d_fsframe.transpose()<<std::endl;
    // std::cout<<"pose_task2d_tcp = "<<pose_task2d_tcp.transpose()<<std::endl;
    LinY2X2D(pose_fsframe_tcp,pose_fsframe_tcp_l);
    //std::cout<<"pose_fsframe_tcp = "<<pose_fsframe_tcp.transpose()<<std::endl;

    //6. 得到人机交互的前馈速度，该前馈速度可以作为独轮车模型的预瞄点
    //std::cout<<"Run to here 6"<<std::endl;
    Eigen::Matrix<double,3,1> task2d_vel;
    AdmittanceMatrix(pose_base_tcp, pose_base_task, fh, task2d_vel);
    

    //7. 控制器
    //std::cout<<"Run to here 7"<<std::endl;
    Eigen::Matrix<double,3,1> _u;
    Controller(pose_fsframe_tcp_l,task2d_vel, _u);
    
    
    //8. 还原到高阶
    //std::cout<<"Run to here 8"<<std::endl;
    // std::cout<<"theta = "<<theta<<std::endl;
    tcp_vel.setZero();
    tcp_vel(0) = _u(0);
    tcp_vel(1) =  0.0;
    tcp_vel(5) = _u(2);

    Eigen::Matrix<double,7,1> state_task;
    state_task<<_u,task2d_vel,theta;
    saveData(std::string("/home/agr/catkin_ws/state_task.csv"), state_task.transpose());

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
    usleep(5000);
    /****TIME CONSUMING****/
}


int PathFollowTask::DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1> fe,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta)
{
   DynamicPlanVel(pose_base_tcp, pose_base_task,fh,tcp_vel,theta);

   return 1;
}



static int iii = 0;
// Abuse
int PathFollowTask::DimReductionTargetFrame(Eigen::Matrix<double,6,1> pose_task_tcp,Eigen::Matrix<double,3,1>& pose_task2d_tcp)
{
    //1.没有用到！！！！ 将task3d坐标系转换到（PCA）task2d坐标系上
    Eigen::Matrix<double,4,4> Task_TCP_T;
    Task_TCP_T.setZero();
    PoseToTransform(pose_task_tcp,Task_TCP_T);

    Eigen::Matrix<double,4,4> task2d_tcp_T = baseFrame*Task_TCP_T;

    // 将SE3矩阵转换成旋量
    Eigen::Matrix<double,6,1> pose_task2d_tcp3;
    TransformToPose(task2d_tcp_T,pose_task2d_tcp3);

    //2. SE3空间变为SE2空间
    pose_task2d_tcp = DimReductionMatrix*pose_task2d_tcp3;
}


int PathFollowTask::DimReductionTargetFrame(Eigen::Matrix<double,4,4> Task_TCP_T,Eigen::Matrix<double,3,1>& pose_task2d_tcp)
{
    //1. 将task3d坐标系转换到（PCA）task2d坐标系上

    Eigen::Matrix<double,4,4> task2d_tcp_T = baseFrame*Task_TCP_T;

    // 将SE3矩阵转换成旋量
    Eigen::Matrix<double,6,1> pose_task2d_tcp3;
    TransformToPose(task2d_tcp_T,pose_task2d_tcp3);

    //2. SE3空间变为SE2空间
    pose_task2d_tcp = DimReductionMatrix*pose_task2d_tcp3;
    //std::cout<< "[3]pose_task2d_tcp3 = "<<pose_task2d_tcp3.transpose()<<std::endl;

    Eigen::Matrix3d R_task2d_tcp = task2d_tcp_T.block(0,0,3,3);
    Eigen::AngleAxisd rotation_vec(R_task2d_tcp);
    
    
    //std::cout<< "[4]rotation_vec = "<<rotation_vec.angle()<<std::endl;
    Eigen::Vector3d v3d =rotation_vec.axis();
    //std::cout<< "[5]rotation_vec = "<<v3d(0)<<", "<<v3d(1)<<", "<<v3d(2)<<", "<<std::endl;

    // if(v3d(2)>=0)
    // {
        pose_task2d_tcp(2) = rotation_vec.angle();
    // }
    // else
    // {
    //     pose_task2d_tcp(2) = 2*M_PI-rotation_vec.angle();
    //     // pose_task2d_tcp(0) = - pose_task2d_tcp(2);
    // }

}

//相对于2d的世界坐标系下
int PathFollowTask::LinY2X2D(Eigen::Matrix<double,3,1> y,Eigen::Matrix<double,3,1>& x)
{
    //1. 线性化，独轮车模型
    //std::cout<<"y = "<<y<<std::endl;
    Eigen::Matrix<double,3,1> params;
    params << cos(y(2)),sin(y(2)),0;
    x = y+ _b*params;
    return 1;
}

//Abuse
int PathFollowTask::FrenetSerretFrame(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta)
{
    // 利用LM算法求解 FS标架问题
    //1. 给定y ,预瞄特性
    Eigen::Matrix<double,2,1> yy(y(0)+2.0*_b*cos(y(2)),y(1)+2.0*_b*sin(y(2)));
    cost_function = new QuadraticCostFunction(this,yy);

    double lower_bound = 0.0;

    double upper_bound = 1.0;
    double theta_last = theta;


    // 设置y
    problem.AddResidualBlock(cost_function, NULL,&theta);
    //2. 求解优化问题，得到待优化变量theta
    Solve(options, &problem, &summary);

    if((theta<=1)&&(theta>=0))
    {
        theta_last = theta;
    }
    else
    {
        theta = theta_last;
        std::cout<<"Update Failed  "<<theta<<std::endl;
    }

    // 通过广义坐标计算代理点位置
    FromThetaToAgent(theta,pose);

    return 1;
}

int PathFollowTask::FrenetSerretFrame_Plain(Eigen::Matrix<double,3,1> y, Eigen::Matrix<double,3,1>& pose, double& theta)
{
    //1. 给定y 
    Eigen::Matrix<double,2,1> yy(y(0)+2.0*_b*cos(y(2)),y(1)+2.0*_b*sin(y(2)));
    
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
        if(Jc-min_Jc<0.0)
        {
            min_s = s;
            min_Jc = Jc;
        }
    }
    theta = min_s*0.001;

    if(theta_last>theta)
    {
        theta = theta_last;
    }


    FromThetaToAgent(theta,pose);

    return 1;


    // while((Jaco<threshold_stop)&&(k<iter_num))
    // {
    //     k++;
    //     Jo = J(theta,yy);
    //     Jaco = JDot2(theta,yy);
    //     delta_theta = alpha*(Jo*Jaco)/(Jaco*Jaco+0.001);
    //     theta = delta_theta+theta;
    // }
    
}

int PathFollowTask::findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose)
{
    Eigen::Matrix<double,3,1> y;
    FromThetaToAgent(0.0,y);

    //这个地方的task3d实际上是task2d 在3d坐标下的表示

    Eigen::Matrix<double,6,1> pose_task3d_cutP;
    pose_task3d_cutP.setZero();
    pose_task3d_cutP(0) = y(0)-0.01*cos(y(2));
    pose_task3d_cutP(1) = y(1)-0.01*sin(y(2));
    //     pose_task3d_cutP(0) = y(0);
    // pose_task3d_cutP(1) = y(1)+0.003;

    Eigen::Matrix<double,6,1> p_judge;
    TransformToPose(baseFrame,p_judge);
    if(p_judge(5)>0)
    {
        pose_task3d_cutP(2) = 0.05;
    }
    else
    {
        pose_task3d_cutP(2) = -0.05;
    }

    pose_task3d_cutP(5) = y(2);
    Eigen::Matrix<double,4,4> T_task3d_cutP;
    PoseToTransform(pose_task3d_cutP,T_task3d_cutP);


    Eigen::Matrix<double,4,4> T_marker_cutP = baseFrame.inverse()*T_task3d_cutP;

    Eigen::Matrix<double,6,1> p;
    TransformToPose(T_marker_cutP,p);

    pose = p;
    return 1;

}

int PathFollowTask::FromThetaToAgent(double theta, Eigen::Matrix<double,3,1>& y)
{
    assert((theta>=0)&&(theta<=1));
    Eigen::Matrix<double,2,1> y1;
    sigma(theta,y1);

    
    Eigen::Matrix<double,2,1> yDot;
    sigmaDot(theta, yDot);

    Eigen::Matrix<double,3,1> xx;
    xx.setZero();
    xx.block(0,0,2,1) = 1/(yDot.norm())*yDot;

    Eigen::Matrix<double,3,1> xx_base;
    xx_base<<1,0,0;

    Eigen::Map<Eigen::Vector3d> _xx = Eigen::Vector3d::Map(xx.data(),xx.size());
    Eigen::Map<Eigen::Vector3d> _xx_base = Eigen::Vector3d::Map(xx_base.data(),xx_base.size());

    double _xx_dot = _xx.dot(_xx_base);
    Eigen::VectorXd _xx_cross= _xx_base.cross(_xx);
    double thetad = acos(_xx_dot);
    if(_xx_cross[2]<0)
    {
        thetad = 2*M_PI-abs(thetad);
    }

    y.block(0,0,2,1) = y1;
    y(2) = thetad; 

    return 1;

}


int PathFollowTask::AdmittanceMatrix(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,
                                        Eigen::Matrix<double,6,1> twist, Eigen::Matrix<double,3,1>& vel)
{
    //将3dtcp速度投影到2d任务空间下
    // Eigen::Matrix<double,4,4> Base_TCP_T;
    // Base_TCP_T.setZero();
    // PoseToTransform(Base_TCP,Base_TCP_T);

    // Eigen::Matrix<double,4,4> Base_Task_T;
    // Base_Task_T.setZero();
    // PoseToTransform(Base_Task,Base_Task_T);

    // Eigen::Matrix<double,4,4> TCP_Task2d_T = Base_TCP_T.inverse()*Base_Task_T*baseFrame;

    // Eigen::Matrix<double,6,6> TCP_Task2d_tTransM;
    // TwistTransMatrix(TCP_Task2d_T,TCP_Task2d_tTransM);

    // Eigen::Matrix<double,6,1> twist_task2d = TCP_Task2d_tTransM*twist;

    Eigen::Matrix<double,3,1> vel_temp;
    vel_temp.setZero();
    vel_temp(0)=twist(0);
    vel_temp(1)=twist(1);

    //选择旋转轴
    vel_temp(2)=twist(5);

    vel = admittance_human_guide*vel_temp;



    //std::cout<<"twist_task2d = "<<twist_task2d.transpose()<<std::endl;
    // vel(0) =0.0;
    // vel(1) =0.0;
    // vel(2) =0.0;
    
    return 1;
}

int PathFollowTask::Controller(Eigen::Matrix<double,3,1> pose_fsframe_tcp_l,  
                                Eigen::Matrix<double,3,1> cmd, Eigen::Matrix<double,3,1>& u)
{
    //1. u 是速度，对应了Target 2d坐标系下的速度
    // Eigen::Matrix<double,3,3> pose_task_fsframe_T;
    // TransformToPose2D(pose_task_fsframe,pose_task_fsframe_T);

    // Eigen::Matrix<double,3,3> pose_task_tcp_l_T;
    // TransformToPose2D(pose_task_tcp_l,pose_task_tcp_l_T);

    // Eigen::Matrix<double,3,3> pose_fsframe_tcp_l_T = pose_task_fsframe_T.inverse()*pose_task_tcp_l_T;

    Eigen::Matrix<double,2,1> p;
    p<< pose_fsframe_tcp_l(0),pose_fsframe_tcp_l(1);

    double theta = pose_fsframe_tcp_l(2);

    Eigen::Matrix<double,3,3> pose_fsframe_tcp_l_tTransM;

    Eigen::Matrix<double,3,3> pose_fsframe_tcp_l_T;
    PoseToTransform(pose_fsframe_tcp_l,pose_fsframe_tcp_l_T);

    TwistTransMatrix(pose_fsframe_tcp_l_T,pose_fsframe_tcp_l_tTransM);
    // Eigen::Matrix<double,3,1> fsframe_tcp_vel_human_guide = pose_fsframe_tcp_l_tTransM*admittance_human_guide*cmd;

    Eigen::Matrix<double,3,1> fsframe_tcp_vel_human_guide = cmd;
    // Eigen::Matrix<double,3,1> fsframe_tcp_vel_human_guide = admittance_human_guide*cmd;


    // std::cout<<"fsframe_tcp_vel_human_guide = "<<fsframe_tcp_vel_human_guide.transpose()<<std::endl;
    // fsframe_tcp_vel_human_guide(0) = speed_const;
    fsframe_tcp_vel_human_guide(1) =0.0;
    fsframe_tcp_vel_human_guide(2) =0.0;


    Eigen::Matrix<double,2,2> M;
    M<<cos(theta),sin(theta),-sin(theta)/_b,cos(theta)/_b;
    Eigen::Matrix<double,2,1> vw;
    if(fsframe_tcp_vel_human_guide(0)>0.0004)
    {
        vw =  M*(Kp*p+Kd*(p-p_t_1)/0.01 + fsframe_tcp_vel_human_guide.block(0,0,2,1));
    } 
    else
    {
        vw.setZero();
    }

    p_t_2 = p_t_1;
    p_t_1 = p;

    // std::cout<<"vw = "<<vw.transpose()<<std::endl;

    if(vw(0)>0.0012)
    {
       // vw(0) = 0.05/vw(1)*vw(0);
        vw(0) = 0.0012; 
    }



    if(vw(1)>0.035)
    {
       // vw(0) = 0.05/vw(1)*vw(0);
        vw(1) = 0.035; 
    }
    else if(vw(1)<-0.035)
    {
        //vw(0) = -0.05/vw(1)*vw(0);
        vw(1) = -0.035;
    }

    // if(vw(0)>0.005)
    // {
    //     vw(0)=0.005;
    // }
    // else if(vw(0)<-0.005)
    // {
    //     vw(0)=-0.005;
    // }

    //TODO 控制量u到tcp上的速度
    u.setZero();
    u(0) = vw(0);
    u(1) =  0.0;
    u(2) = vw(1); 

    return 1;
}

QuadraticCostFunction::QuadraticCostFunction(PathFollowTask* ptrPF,Eigen::Matrix<double,2,1> pt)
{
    _pt.setZero();
    // _p.setZero();
    // _pdt.setZero();
    _ptrPF = ptrPF;
    _pt = pt;
}

QuadraticCostFunction::~QuadraticCostFunction(){}

int QuadraticCostFunction::setCurrentPose(Eigen::Matrix<double,2,1> pt)
{
    _pt = pt;
    return 0;
}


bool QuadraticCostFunction::Evaluate(double const* const* parameters,double* residuals,double** jacobians) const
{

    double theta = parameters[0][0]; // parameters[0]表示取出第一组参数

    Eigen::Matrix<double,2,1> _p;
    _p.setZero();
    _ptrPF->sigma(theta, _p);

    // 计算残差
    //residuals[0]=sqrt(_p(0)*_p(0)+_p(1)*_p(1)-2*_p(0)*_pt(0)-2*_p(1)*_pt(1));
    residuals[1]=_p(1)-_pt(1);
    residuals[0]=_p(0)-_pt(0);


    // 计算雅克比
    Eigen::Matrix<double,2,1> _pd;
    _pd.setZero();
    if (jacobians != NULL && jacobians[0] != NULL) {
        _ptrPF->sigmaDot(theta,_pd);
        jacobians[0][0] = _pd(0);
        jacobians[0][1] = _pd(1);
    }


    return true;
}

// baseFrame = -0.336802  0.822082  0.459069  0.172824
// -0.455932  0.284193 -0.843422 -0.580309
//  0.823826  0.493371 -0.279097 -0.200922
//         0         0         0         1

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

    //时间测试
    static struct timeval start;
    static struct timeval end;
    gettimeofday(&start, NULL);
    for(int i;i<100;i++)
    {
    instance.FrenetSerretFrame(y3, pose3, theta2);
    }
    std::cout<<"!!!pose3 = "<<pose3<<" theta2 = "<<theta2<<std::endl;
    gettimeofday(&end, NULL);
    double timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec; 
    timeuse /= 1000000;
    printf("timeuse = %f\n",timeuse);


    instance.sigma(0.083,y);
     std::cout<<"pose4 = "<<y<<std::endl;

    Eigen::Matrix<double, 6, 1> regist_inform_object_rr;
    Eigen::Matrix<double,4,4> TT_regist = instance.baseFrame;
    std::cout<<"TT_regist ="<<TT_regist<<std::endl;
    TransformToPose(TT_regist,regist_inform_object_rr);
    //std::cout<<"instance.baseFrame= "<<instance.baseFrame<<std::endl;
    std::cout<<"regist_inform_object_rr = "<<regist_inform_object_rr.transpose()<<std::endl;
    Eigen::Matrix<double, 4, 4> regist_inform_object_T;
    PoseToTransform(regist_inform_object_rr,regist_inform_object_T);
    std::cout<<"regist_inform_object_T = "<<regist_inform_object_T<<std::endl;


    Eigen::Matrix<double,6,1> Base_tcp;
    //Base_tcp<<0.0587335,  -0.544038,   0.631389, -0.0605423,   0.877072,    2.67187;
    //Base_tcp<<0.0698,  -0.59999,   0.549772, 0.0,   0.0,    -3.14;
    //Base_tcp<<0.05,  -0.59999,   0.6, 0.0,   0.0,    1.571;

    Base_tcp<<0.55,  -0.34,   0.40, 0.0,   0.0,    -0.8;




   // Base_tcp<<0.104428,  -0.675247,   0.575827,   0.239909, -0.0735479,    2.85173;
    //Base_tcp<<0.104428,  -0.675247,   0.575827,   0.0,1.00,1.0;

    //Base_tcp<<0.104428,  -0.675247,   0.575827,0.239909, -0.0735479,    2.85173;
    Eigen::Matrix<double,4,4> TT;
    PoseRPY2Transform(Base_tcp,TT);



    Eigen::Matrix<double, 6, 1> reg_cc;


    Eigen::Matrix<double,4,4> TT_out =TT*regist_inform_object_T;
    //Eigen::Matrix<double,4,4> TT_out =TT*instance.baseFrame;
    
    // Transform2PoseRPY(TT_out,reg_cc);
    std::cout<<"TT = "<<TT<<std::endl;
    std::cout<<"instance.baseFrame = "<<instance.baseFrame<<std::endl;
    std::cout<<"111 TT_out = "<<TT_out<<std::endl;
    // std::cout<<"reg_cc = "<<reg_cc.transpose()<<std::endl;

    // PoseToTransform(reg_cc,TT_out);
    // std::cout<<"222 TT_out = "<<TT_out<<std::endl;

    TransformToPose(TT_out,reg_cc);
    PoseToTransform(reg_cc,TT_out);

    // std::cout<<"33 TT_out = "<<TT_out<<std::endl;




    // Eigen::Quaterniond quaternion;
    // Eigen::Matrix<double,3,1> rr;
    // rr<<regist_inform_object_rr(3),regist_inform_object_rr(4),regist_inform_object_rr(5);
    // EulerRPY2Quat(rr,quaternion);

    Eigen::Matrix<double,7,1> pose4;
    // //pose4<<regist_inform_object_rr(0),regist_inform_object_rr(1),regist_inform_object_rr(2),quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z();
    AxisAnglePose2QuatPose(reg_cc,pose4);
    std::cout<<"pose4 = "<<pose4.transpose()<<std::endl;
    // // Eigen::Matrix<double,6,1> dist_;
    // Eigen::Matrix<double,6,1> pose5;
    // QuatPose2EulerPoseRPY(pose4,pose5);
    // std::cout<<"pose5 = "<<pose5.transpose()<<std::endl;
    // // dist_<<0.038

    //Eigen::Matrix<double,4,4> TT1 = instance.baseFrame*TT_out.inverse()*TT;
    Eigen::Matrix<double,4,4> TT1 = regist_inform_object_T*TT_out.inverse()*TT;
    std::cout<<"TT1 = "<<TT1<<std::endl;

        Eigen::Matrix<double,4,4> TT2 = TT_out.inverse()*TT;
    std::cout<<"TT2 = "<<TT2<<std::endl;
    std::cout<<"regist_inform_object_T = "<<regist_inform_object_T<<std::endl;
    std::cout<<"instance.baseFrame = "<<instance.baseFrame<<std::endl;


    





    return 0;
    
}





