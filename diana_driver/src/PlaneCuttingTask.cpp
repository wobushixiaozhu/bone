#include "PlaneCuttingTask.hpp"

//TODO:TwistTransMatrix,PoseToTransform,TwistTransMatrix2d
//TODO :DimReductionMatrix,PoseTransform2D

PlaneCuttingTask::PlaneCuttingTask(char* name)
{
    int ret = findbaseFrame(name, baseFrame);
    std::cout<<"findBaseFrame = "<<ret<<std::endl;
}

PlaneCuttingTask::~PlaneCuttingTask()
{
    //delete cost_function;
}


static int iii = 0;

int PlaneCuttingTask::findbaseFrame(char* name, Eigen::Matrix<double,4,4>& baseFrame)
{
    Eigen::MatrixXd OriMatrix;
    importPathFromFile(name,OriMatrix);
    std::cout<<"OriMatrix:="<<OriMatrix<<std::endl;
    int N = OriMatrix.cols();
    std::cout<<"N:="<<N<<std::endl;


    Eigen::Matrix<double,3,3> M;
    M<<OriMatrix(0,0),OriMatrix(0,1),OriMatrix(0,2),
        OriMatrix(1,0),OriMatrix(1,1),OriMatrix(1,2),
        OriMatrix(2,0),OriMatrix(2,1),OriMatrix(2,2);


    double xx13 = OriMatrix(0,2)-OriMatrix(0,0);
    double yy13 = OriMatrix(1,2)-OriMatrix(1,0);
    double zz13 = OriMatrix(2,2)-OriMatrix(2,0);
    Eigen::Vector3d x_axis(xx13,yy13,zz13);
    double rxx = x_axis.norm();

    x_axis = x_axis/rxx;

    double xx12 = OriMatrix(0,1)-OriMatrix(0,0);
    double yy12 = OriMatrix(1,1)-OriMatrix(1,0);
    double zz12 = OriMatrix(2,1)-OriMatrix(2,0);

    // Eigen::Matrix<double,3,3> Ay;
    // Ay<<xx12,yy12,zz12,xx13,yy13,zz13,0.0,0.0,0.0;

    //Ay*Xy = 0;


    // Eigen::Matrix<double,3,1> yy = Ay.llt().solve(Eigen::MatrixXd::Zero(3,1));
    Eigen::Vector3d z_axis_temp(xx12,yy12,zz12);



    Eigen::Vector3d y_axis = z_axis_temp.cross(x_axis);
    double ryy = y_axis.norm();
    y_axis = y_axis/ryy;


    Eigen::Vector3d z_axis = x_axis.cross(y_axis);
    double rzz = z_axis.norm();
    z_axis = z_axis/rzz;

    if(rzz < 1e-4)
    {
        std::cout<<"The first 3 points are colinear in geometry. Pls select points again!"<<std::endl;    
        std::cout<<"resutl = "<<rzz<<std::endl;
        exit(0);
    }
    std::cout<<"y_axis = "<<y_axis<<std::endl;
    std::cout<<"z_axis = "<<z_axis<<std::endl;

    if(z_axis(2)<0)
    {
        z_axis = -1.0*z_axis;
        y_axis = -1.0*y_axis;
    }

    baseFrame.setZero();
    Eigen::Matrix<double,3,3> RR;
    RR<<x_axis,y_axis,z_axis;
    std::cout<<"RR = "<<RR<<std::endl;

    baseFrame.setZero();
    baseFrame.block(0,0,3,3) = RR;
    baseFrame(3,3) = 1.0;

    baseFrame.block(0,3,3,1)<<OriMatrix(0,0),OriMatrix(1,0),OriMatrix(2,0);

    std::cout<<"baseFrame = "<<baseFrame<<std::endl;
    


}


int PlaneCuttingTask::findInitCuttingPoint(Eigen::Matrix<double,6,1>& pose)
{
    //设计进刀点
    Eigen::Matrix<double,6,1> pose_task3d_cutP;
    pose_task3d_cutP.setZero();
    pose_task3d_cutP(0) = -0.015;
    pose_task3d_cutP(2) = 0.03;

    Eigen::Matrix<double,4,4> T_task3d_cutP;
    PoseToTransform(pose_task3d_cutP,T_task3d_cutP);


    Eigen::Matrix<double,4,4> T_marker_cutP = baseFrame.inverse()*T_task3d_cutP;

    Eigen::Matrix<double,6,1> p;
    TransformToPose(T_marker_cutP,p);

    pose = p;

    return 1;

}


int PlaneCuttingTask::DynamicPlanVel(Eigen::Matrix<double,6,1> pose_base_tcp, 
                    Eigen::Matrix<double,6,1> pose_base_task, 
                    Eigen::Matrix<double,6,1> fh,
                    Eigen::Matrix<double,6,1>& tcp_vel,
                    double& theta)
{
    //动态规划，输入变量为头颅坐标系注册信息，机械臂基座到tcp的信息，tcp下的力信息； 输出为tcp下的速度信息
    //1. 将机械臂TCP坐标系转换到Task下
    //std::cout<<"Run to here 1"<<std::endl;

    Eigen::DiagonalMatrix<double,6> Admittance;

    //Admittance.diagonal() << 0.05,0.05,0.05,0.00,0.00,0.1;  
    //Admittance.diagonal() << 0.01,0.06,0.02,0.1,0.1,0.1;

    Eigen::Matrix<double,6,1> pose_tcp_task;
    PoseTransform3D(pose_base_tcp,pose_base_task,pose_tcp_task);

    double zz = pose_tcp_task(2);
    double fz = fh(2);


    double scale = 1.0;
    if (fz< 0.1)
    {
        scale = 1/(1+exp(-2.0*(7e2*zz)));
    }
    else
    {
        scale = 1.0;
    }
    //std::cout<<"zz = "<<zz<<"; fz ="<<fz<<"; scale ="<<scale<<std::endl;
    
    Admittance.diagonal() << 0.05,0.00,0.05*scale,0.0,0.0,0.0;   

    tcp_vel=Admittance*fh;

    //Cyber 2022
    static int itk_cyber = 0;
    
    itk_cyber++;
    if(itk_cyber/10 == 0)
    {
        saveData(std::string("/home/agr/catkin_ws/fh.csv"),fh.transpose());
        saveData(std::string("/home/agr/catkin_ws/pose_base_tcp.csv"),pose_base_tcp.transpose());
        saveData(std::string("/home/agr/catkin_ws/pose_tcp_task.csv"),pose_tcp_task.transpose());
        saveData(std::string("/home/agr/catkin_ws/tcp_vel.csv"),tcp_vel.transpose());
        Eigen::Matrix<double,1,1> admittancek;
        admittancek<< 0.05*scale;
        std::cout<<"zz = "<<zz<<"; fz ="<<fz<<"; scale ="<<scale<<std::endl;
    
        saveData(std::string("/home/agr/catkin_ws/admittancek.csv"),admittancek.transpose());
        itk_cyber=0;
    }



    return 1;

    /****TIME CONSUMING****/
}




void PlaneCuttingTask::importPathFromFile(char* name,Eigen::MatrixXd& T)
{
        //1. 读取文件内容信息
    std::ifstream fin(name);

    if(!fin.is_open())
    {
        std::cout<<"Fail to open the file!"<<std::endl;
        //return;
    }


    //2. 正则表达式
    //std::regex reg1(R"(^([X]\d\s[Y]\d\s[Z]\d\:)([\s]+-?[0-9]+(.[0-9]{2})?)+$)");
    std::regex reg1(R"(^([X]\d\s[Y]\d\s[Z]\d\:)([\s]+-?[0-9]+.[0-9]{2}?)([\s]+-?[0-9]+.[0-9]{2}?)([\s]+-?[0-9]+.[0-9]{2}?))");
    std::smatch m;
    //
    bool ret;
    std::string buf;
    //3. 循环，读TXT中的数据内容
    Eigen::MatrixXd OriMatrix;
    OriMatrix.resize(3,0);

    while (std::getline(fin, buf))
    {
        std::cout << buf << std::endl;
        std::string temp = buf;
        ret = std::regex_search(temp,m,reg1);

        std::cout << "----------------"<<m[0] <<"  "<<ret<< std::endl;
        if(ret&&m.size()==5)
        {
            // std::cout<<"E:"<<m.str(2)<<"  "<<m.str(3)<<"  "<<m.str(4)<<std::endl;
             OriMatrix.conservativeResize(OriMatrix.rows(), OriMatrix.cols()+1);
             OriMatrix.block(0,OriMatrix.cols()-1,3,1)<<
             std::stod(trim(m.str(2)))/1000.0,
             std::stod(trim(m.str(3)))/1000.0,
             std::stod(trim(m.str(4)))/1000.0;

        }
     

    }

    T = OriMatrix;
    std::cout << "----------------" << std::endl;
}

int main(int argc, char* argv[])
{
    
    return 0;
    
}





