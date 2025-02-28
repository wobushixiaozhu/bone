#include "utility_med.h"

int Skew_c(const Eigen::Matrix<double, 3, 1> omega,Eigen::Matrix<double, 3, 3>& Omega){
    // Eigen::Matrix<double, 3, 3> Omega;
    Omega.setZero();
    Omega(0,1)=-omega[2];
    Omega(0,2)= omega[1];
    Omega(1,0)= omega[2];
    Omega(1,2)=-omega[0];
    Omega(2,0)=-omega[1];
    Omega(2,1)= omega[0];

    return 1;

}

int Skew_inv(const Eigen::Matrix<double, 3, 3> Omega, Eigen::Matrix<double, 3, 1>& omega){

    omega[1] = Omega(0,2);
    omega[2] = Omega(1,0);
    omega[0] = Omega(2,1);

    return 1;

}

void importPointFromFile(std::string name,Eigen::MatrixXd& T)
{
    char* name_c = (char* )name.c_str();
    importPointFromFile(name_c, T);

}

void importPointFromFile(char* name,Eigen::MatrixXd& T)
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
    std::regex reg1(R"(^([X][\d]+\s[Y][\d]+\s[Z][\d]+\:)([\s]+-?[0-9]+.[0-9]{2}?)([\s]+-?[0-9]+.[0-9]{2}?)([\s]+-?[0-9]+.[0-9]{2}?))");
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

 

std::string trim(std::string str){

    str.erase(0, str.find_first_not_of(" \t")); // 去掉头部空格

    str.erase(str.find_last_not_of(" \t") + 1); // 去掉尾部空格

    return str;
}



bool isRotationMatirx(Eigen::Matrix3d R)
{
    double err=1e-6;
    Eigen::Matrix3d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
    return (shouldIdenity - I).norm() < err;
}

bool isRotationMatirx(Eigen::Matrix2d R)
{
    double err=1e-6;
    Eigen::Matrix2d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix2d I=Eigen::Matrix2d::Identity();
    //std::cout<<"R = "<<R<<std::endl;
    return (shouldIdenity - I).norm() < err;
}

int rotationMatrixToEulerAngles(Eigen::Matrix3d R, Eigen::Matrix<double,3,1>& theta)
{
    assert(isRotationMatirx(R));
    // double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    // bool singular = sy < 1e-6;
    // double x, y, z;

    // if (!singular)
    // {
    //     x = atan2( R(2,1), R(2,2));
    //     y = atan2(-R(2,0), sy);
    //     z = atan2( R(1,0), R(0,0));
    // }
    // else
    // {
    //     x = atan2(-R(1,2), R(1,1));
    //     y = atan2(-R(2,0), sy);
    //     z = 0;
    // }
    double roll, pitch, yaw;
    tf::Matrix3x3 RR;
    RR.setValue(R(0,0),R(0,1),R(0,2),
                R(1,0),R(1,1),R(1,2),
                R(2,0),R(2,1),R(2,2));

    RR.getRPY(roll,pitch,yaw);
    // Eigen::Vector3d eulerAngle=R.eulerAngles(2,1,0);
    theta(0) = roll;
    theta(1) = pitch;
    theta(2) = yaw;

    return 1;
}

int rotationMatrixToEulerAngles(Eigen::Matrix2d R, double& theta)
{
    //std::cout<<"RRR = "<<R<<std::endl;
    assert(isRotationMatirx(R));

    double sy = sqrt(R(0,0) * R(0,0));
    bool singular = sy < 1e-6;

    if (!singular)
    {
        theta = atan2( R(1,0), R(0,0));
    }
    else
    {
        //theta = atan2(-R(1,2), R(1,1));
        if(R(1,0)>0.0)
            theta = M_PI/2;
        else
            theta = -M_PI/2;
        
    }

    return 1;
}


int WrenchTransMatrix(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,6>& A_B_tTransM)
{

    Eigen::Matrix<double, 3, 3> R_RoTohh = A_B_T.block(0,0,3,3);
    Eigen::Matrix<double, 3, 1> t_RoTohh = A_B_T.block(0,3,3,1);
    
    A_B_tTransM.setZero();
    A_B_tTransM.topLeftCorner(3,3) = R_RoTohh;
    A_B_tTransM.bottomRightCorner(3,3) = R_RoTohh;
    Eigen::Matrix<double,3,3> RR_temp;
    Skew_c(t_RoTohh,RR_temp);
    // std::cout<<"t_RoTohh = "<<t_RoTohh<<std::endl;
    // std::cout<<"RR_temp = "<<RR_temp<<std::endl;
    A_B_tTransM.bottomLeftCorner(3,3) = RR_temp*R_RoTohh;

    return 1;

}



int TwistTransMatrix(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,6>& A_B_tTransM)
{
   Eigen::Matrix<double,4,4> B_A_T = A_B_T.inverse();
   A_B_tTransM.setZero();
   A_B_tTransM.block(0,0,3,3) =A_B_T.block(0,0,3,3);
   A_B_tTransM.block(3,3,3,3) =A_B_T.block(0,0,3,3);
   Eigen::Matrix<double,3,1> B_A_ORI_P = B_A_T.block(0,3,3,1);

   Eigen::Matrix<double,3,3> R_temp;
   Skew_c(B_A_ORI_P,R_temp);
   A_B_tTransM.block(0,3,3,3) = -A_B_T.block(0,0,3,3)*R_temp;

   return 1;

}

int TwistTransMatrix(Eigen::Matrix<double,3,3> A_B_T,Eigen::Matrix<double,3,3>& A_B_tTransM)
{
    A_B_tTransM.setZero();
    A_B_tTransM.block(0,0,2,2) =A_B_T.block(0,0,2,2);
    A_B_tTransM.block(0,2,2,1) = -A_B_T.block(0,2,2,1);
    A_B_tTransM(2,2) = 1.0;
   return 1;
}

int PoseToTransform(Eigen::Matrix<double,6,1> A_B,Eigen::Matrix<double,4,4>& A_B_T)
{
    A_B_T.setZero();
    Eigen::Vector3d v(A_B(3),A_B(4),A_B(5));
    double r = v.norm();
    // double x = A_B(3);
    // double y = A_B(4);
    // double z = A_B(5);
    if(r>1e-6){
        v = v/r;
    }
    else
    {
        v(0) = 0.0;
        v(1) = 0.0;
        v(2) = 0.0;
    }

    // Eigen::Matrix3f m;
    //     m = Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitX())
    //     * Eigen::AngleAxisf(0.5*M_PI,  Eigen::Vector3f::UnitY())
    //     * Eigen::AngleAxisf(0.33*M_PI, Eigen::Vector3f::UnitZ());

//     Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
// Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
// Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));
 

    Eigen::AngleAxisd Rv(r, v);
    Eigen::Matrix3d R = Rv.toRotationMatrix();
    // R = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())*
    //                                  Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())*
    //                     Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());

    //  tf::Matrix3x3 RR;
    




    //  RR.setRPY(x,y,z);
    // R<<RR[0].x(),RR[0].y(),RR[0].z(),
    // RR[1].x(),RR[1].y(),RR[1].z(),
    // RR[2].x(),RR[2].y(),RR[2].z();

    A_B_T.block(0,0,3,3) = R; 

    A_B_T.block(0,3,3,1) = A_B.block(0,0,3,1);
    A_B_T(3,3) = 1;

    return 1;
}

int PoseToTransform(Eigen::Matrix<double,3,1> A_B,Eigen::Matrix<double,3,3>& A_B_T)
{
    double theta = A_B(2);
    A_B_T.setZero();
    A_B_T.block(0,0,2,2)<<cos(theta),-sin(theta),sin(theta),cos(theta); 
    A_B_T.block(0,2,3,1)<<A_B(0),A_B(1),1.0;
    return 1;
}

int TransformToPose(Eigen::Matrix<double,3,3> A_B_T,Eigen::Matrix<double,3,1>& A_B)
{
    A_B.block(0,0,2,1) = A_B_T.block(0,2,2,1);
    Eigen::Matrix2d R = A_B_T.block(0,0,2,2);
    double theta = 0.0;
    rotationMatrixToEulerAngles(R,theta);
    A_B(2) = theta; 
    return 1;
}

// int TransformToPose(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,1>& A_B)
// {
//     A_B.block(0,0,3,1) = A_B_T.block(0,3,3,1);
//     Eigen::Matrix3d R = A_B_T.block(0,0,3,3);
//     Eigen::Matrix<double,3,1> theta;
//     theta<<0.0,0.0,0.0;
//     rotationMatrixToEulerAngles(R,theta);
//     //A_B(2) = theta;
//     A_B(3) = theta(0);
//     A_B(4) = theta(1);
//     A_B(5) = theta(2); 
//     return 1;
// }


int PoseTransform2D(Eigen::Matrix<double,3,1> pose_A_B,Eigen::Matrix<double,3,1> pose_A_C,Eigen::Matrix<double,3,1>& pose_B_C)
{
    Eigen::Matrix<double,3,3> pose_A_B_T;
    Eigen::Matrix<double,3,3> pose_A_C_T;
    // std::cout<<"pose_A_B_T = "<<pose_A_B_T<<std::endl;
    // std::cout<<"pose_A_C_T = "<<pose_A_C_T<<std::endl;
    PoseToTransform(pose_A_B,pose_A_B_T);
    PoseToTransform(pose_A_C,pose_A_C_T);

    Eigen::Matrix<double,3,3> pose_B_C_T = pose_A_B_T.inverse()*pose_A_C_T;

    TransformToPose(pose_B_C_T,pose_B_C);

    return 1;
}

int rotationMatrixToAxisAngle(Eigen::Matrix<double,3,3> R,Eigen::Matrix<double,3,1>& r)
{
    Eigen::AngleAxisd rotation_vector(R);
    Eigen::Vector3d v = rotation_vector.axis();
    double rr = rotation_vector.angle();
    v = rr*v;

    r(0) = v(0);
    r(1) = v(1);
    r(2) = v(2);

}




int TransformToPose(Eigen::Matrix<double,4,4> A_B_T,Eigen::Matrix<double,6,1>& A_B)
{
    Eigen::Matrix3d R = A_B_T.block(0,0,3,3);
    Eigen::Matrix<double,3,1> theta;
    rotationMatrixToAxisAngle(R,theta);

    
    //A_B.block(0,0,3,1) =A_B_T.block(0,3,3,1);
    A_B.block(0,0,3,1) = A_B_T.block(0,3,3,1);
    //std::cout<<"run to here  "<<theta<<std::endl;
    A_B.block(3,0,3,1) = theta;

    return 1;

}


int EulerRPY2Quat(Eigen::Matrix<double,3,1> eulerAngle,Eigen::Matrix<double,4,1>& quaternion)
{
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
 
    // quaternion=yawAngle*pitchAngle*rollAngle;
    tf::Quaternion q;
    q.setRPY(eulerAngle(0),eulerAngle(1),eulerAngle(2));
    quaternion(0) = q.w();
    quaternion(1) = q.x();
    quaternion(2) = q.y();
    quaternion(3) = q.z();
    return 1;
}

int AxisAnglePose2QuatPose(Eigen::Matrix<double,6,1> pose_a,Eigen::Matrix<double,7,1>& pose_q)
{
    Eigen::Matrix<double,4,1> quaternion;
    Eigen::Matrix<double,3,1> rr;
    rr<<pose_a(3),pose_a(4),pose_a(5);

    AxisAngle2Quat(rr,quaternion);

    pose_q<<pose_a(0),pose_a(1),pose_a(2),quaternion(0),quaternion(1),quaternion(2),quaternion(3);

    return 1;


}

int AxisAngle2Quat(Eigen::Matrix<double,3,1> AxisAngle,Eigen::Matrix<double,4,1>& quaternion)
{
    Eigen::Vector3d v(AxisAngle(0),AxisAngle(1),AxisAngle(2));
    double r = v.norm();
    if(r>1e-6){
        v = v/r;
    }
    else
    {
        v(0) = 0.0;
        v(1) = 0.0;
        v(2) = 0.0;
    }
    Eigen::AngleAxisd rotation_vector(r,v);

    Eigen::Quaterniond quaterniond_q(rotation_vector);
    quaternion(0) = quaterniond_q.w();
    quaternion(1) = quaterniond_q.x();
    quaternion(2) = quaterniond_q.y();
    quaternion(3) = quaterniond_q.z();
    return 1;
}

int Quat2AxisAngle(Eigen::Matrix<double,4,1> quaternion,Eigen::Matrix<double,3,1>& AxisAngle)
{
    Eigen::Quaterniond q(quaternion(0),quaternion(1),quaternion(2),quaternion(3));

    Eigen::AngleAxisd ax(q);

    Eigen::Vector3d v = ax.axis();
    double r = ax.angle();

    // std::cout<<"vEEEEEEEEEEEEEEE ="<<v<<std::endl;
    // std::cout<<"rEEEEEEEEEEEEEEE ="<<r<<std::endl;

    if(r>1e-6)
    {
        v = v*r;
        AxisAngle(0) = v(0);
        AxisAngle(1) = v(1);
        AxisAngle(2) = v(2);
    }
    else
    {
        AxisAngle(0) = 0.0;
        AxisAngle(1) = 0.0;
        AxisAngle(2) = 0.0;

    }



    return 1;

}

int QuatPose2AxisAnglePose(Eigen::Matrix<double,7,1> pose_q,Eigen::Matrix<double,6,1>& pose_a)
{
        double w = pose_q[3];
    double x = pose_q[4];
    double y = pose_q[5];
    double z = pose_q[6];
    Eigen::Matrix<double,4,1> quaternion;
    quaternion<<w,x,y,z;
    // std::cout<<"quat = "<<quaternion<<std::endl;
    Eigen::Matrix<double,3,1> axisangle;
    Quat2AxisAngle(quaternion,axisangle);
   // std::cout<<"runtohere"<<std::endl;

    //Eigen::Matrix<double,7,1> pose4;
    pose_a<<pose_q(0),pose_q(1),pose_q(2),axisangle(0),axisangle(1),axisangle(2);
    
    return 1;

}
int EulerPoseRPY2QuatPose(Eigen::Matrix<double,6,1> pose_eul,Eigen::Matrix<double,7,1>& pose_q)
{
    Eigen::Matrix<double,4,1> quaternion;
    Eigen::Matrix<double,3,1> rr;
    rr<<pose_eul(3),pose_eul(4),pose_eul(5);
    EulerRPY2Quat(rr,quaternion);

    //Eigen::Matrix<double,7,1> pose4;
    pose_q<<pose_eul(0),pose_eul(1),pose_eul(2),quaternion(0),quaternion(1),quaternion(2),quaternion(3);
    
    return 1;
}



int Quat2EulerRPY(Eigen::Matrix<double,4,1> quaternion,Eigen::Matrix<double,3,1>& eulerAngle)
{
    // Eigen::Quaterniond q(quaternion(0),quaternion(1),quaternion(2),quaternion());
    
    
    
    
    // Eigen::Vector3d ea = quaternion.toRotationMatrix().eulerAngles(2,1,0);
    tf::Quaternion q(quaternion(1),quaternion(2),quaternion(3),quaternion(0));
    // tf::Quaternion q(quaternion(0),quaternion(1),quaternion(2),quaternion(3));
    // q.setRPY(roll,pitch,yaw);

    // std::cout<<"quaternion w = "<<quaternion(0)<<std::endl;
    // std::cout<<"quaternion x = "<<quaternion(1)<<std::endl;
    // std::cout<<"quaternion y = "<<quaternion(2)<<std::endl;
    // std::cout<<"quaternion z = "<<quaternion(3)<<std::endl;

    // std::cout<<"qq w = "<<q.w()<<std::endl;
    // std::cout<<"qq x = "<<q.x()<<std::endl;
    // std::cout<<"qq y = "<<q.y()<<std::endl;
    // std::cout<<"qq z = "<<q.z()<<std::endl;

    double roll,pitch,yaw;
    
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw); 
    eulerAngle(0) = roll;
    eulerAngle(1) = pitch;
    eulerAngle(2) = yaw;

    // tf::Matrix3x3 T = tf::Matrix3x3(q);

    // std::cout<<"T = "<<T[0].x()<< ", "<<T[0].y()<< ", "<<T[0].z()<< ";"<<std::endl;
    // std::cout<<"T = "<<T[1].x()<< ", "<<T[1].y()<< ", "<<T[1].z()<< ";"<<std::endl;
    // std::cout<<"T = "<<T[2].x()<< ", "<<T[2].y()<< ", "<<T[2].z()<< ";"<<std::endl;


    // std::cout<<"roll = "<<roll<<std::endl;

    return 1;
}

int QuatPose2EulerPoseRPY(Eigen::Matrix<double,7,1> pose_q,Eigen::Matrix<double,6,1>& pose_eul)
{
    double w = pose_q[3];
    double x = pose_q[4];
    double y = pose_q[5];
    double z = pose_q[6];
    Eigen::Matrix<double,4,1> quaternion;
    quaternion<<w,x,y,z;
    // std::cout<<"quat = "<<quaternion<<std::endl;
    Eigen::Matrix<double,3,1> eul;
    Quat2EulerRPY(quaternion,eul);
   // std::cout<<"runtohere"<<std::endl;

    //Eigen::Matrix<double,7,1> pose4;
    pose_eul<<pose_q(0),pose_q(1),pose_q(2),eul(0),eul(1),eul(2);
    
    return 1;
}

int PoseRPY2Transform(Eigen::Matrix<double,6,1> A_B, Eigen::Matrix<double,4,4>& A_B_T)
{

    double x = A_B(3);
    double y = A_B(4);
    double z = A_B(5);
 
    tf::Matrix3x3 RR;
    
    A_B_T.setZero();

    Eigen::Matrix<double,3,3> R;

    RR.setRPY(x,y,z);
    R<<RR[0].x(),RR[0].y(),RR[0].z(),
    RR[1].x(),RR[1].y(),RR[1].z(),
    RR[2].x(),RR[2].y(),RR[2].z();

    A_B_T.block(0,0,3,3) = R; 

    A_B_T.block(0,3,3,1) = A_B.block(0,0,3,1);
    A_B_T(3,3) = 1;

    return 1;
}


int rotationMatrixToQuat(Eigen::Matrix<double,3,3> R,Eigen::Matrix<double,4,1>& q)
{
    assert(isRotationMatirx(R));

    Eigen::Quaterniond quaternion(R);

    q(0) = quaternion.w();
    q(1) = quaternion.x();
    q(2) = quaternion.y();
    q(3) = quaternion.z();

    return 1;
}


int PoseTransform3D(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,Eigen::Matrix<double,6,1>& Task_TCP)
{
    //1. 将TCP坐标系转换到Task坐标系下
    Eigen::Matrix<double,4,4> Base_TCP_T;
    Base_TCP_T.setZero();
    PoseToTransform(Base_TCP,Base_TCP_T);
    

    Eigen::Matrix<double,4,4> Base_Task_T;
    Base_Task_T.setZero();
    PoseToTransform(Base_Task,Base_Task_T);
    //std::cout<<"Base_Task = "<<Base_Task.transpose() <<std::endl;

    Eigen::Matrix<double, 4, 4> Task_TCP_T = Base_Task_T.inverse()*Base_TCP_T;
    //std::cout<<"Base_Task_T = "<<Base_Task_T <<std::endl;
    //std::cout<<"Base_Task_T.inverse() = "<<Base_Task_T.inverse() <<std::endl;
    
    //std::cout<<"Base_TCP = "<<Base_TCP <<std::endl;
    //Eigen::Matrix<double,6,1> Task_TCP;
    TransformToPose(Task_TCP_T,Task_TCP);

    return 1;
}

int PoseTransform3D(Eigen::Matrix<double,6,1> Base_TCP, Eigen::Matrix<double,6,1> Base_Task,Eigen::Matrix<double,4,4>& Task_TCP_T)
{
    //1. 将TCP坐标系转换到Task坐标系下
    Eigen::Matrix<double,4,4> Base_TCP_T;
    Base_TCP_T.setZero();
    PoseToTransform(Base_TCP,Base_TCP_T);
    

    Eigen::Matrix<double,4,4> Base_Task_T;
    Base_Task_T.setZero();
    PoseToTransform(Base_Task,Base_Task_T);
    // std::cout<<"Base_Task = "<<Base_Task.transpose() <<std::endl;
    // std::cout<<"Base_TCP = "<<Base_TCP.transpose() <<std::endl;

    Task_TCP_T = Base_Task_T.inverse()*Base_TCP_T;
    // std::cout<<"Base_Task_T = "<<Base_Task_T <<std::endl;
    //std::cout<<"Base_Task_T.inverse() = "<<Base_Task_T.inverse() <<std::endl;
    
    // std::cout<<"Base_TCP_T = "<<Base_TCP_T <<std::endl;
    //Eigen::Matrix<double,6,1> Task_TCP;
    //TransformToPose(Task_TCP_T,Task_TCP);

    return 1;
}



int main()
{
    Eigen::Matrix<double,6,1> pose_cc;
    pose_cc<<-0.231,0.155,0.934,M_PI, M_PI/3, M_PI/6;
    Eigen::Matrix<double,4,4> T1;
    PoseToTransform(pose_cc,T1);

    std::cout<<"pose_cc1 = "<<pose_cc<<std::endl;
    std::cout<<"T1_cc = "<<T1<<std::endl;
    TransformToPose(T1,pose_cc);
    std::cout<<"pose_cc2 = "<<pose_cc<<std::endl;


    Eigen::Matrix<double,6,1> Base_tcp;
    Base_tcp<<0.0587335,  -0.544038,   0.631389, -0.0605423,   0.877072,    2.67187;
    PoseToTransform(Base_tcp,T1);
    
    Eigen::Matrix<double,6,1> regist_inform_object_rr;
    Eigen::Matrix<double,4,4> T2;
    regist_inform_object_rr<<0.172824, -0.580309, -0.200922,   2.08561,  -2.17346,  0.934559;
    PoseToTransform(regist_inform_object_rr,T2);



    Eigen::Matrix<double,4,4> T = T1*T2;
//     Eigen::Matrix<double,4,4> T;
//     T<<0.336802, -0.030016,  0.941097,  0.172824,
// -0.455932, -0.869301, -0.190896, -0.580309,
//  0.823826, -0.493371,  0.279097, -0.200922,
//         0,         0,         0,        1;
    std::cout<<"[1]T = "<<T<<std::endl;
    Eigen::Matrix<double,6,1> pose;
    TransformToPose(T,pose);
    PoseToTransform(pose,T);
    std::cout<<"[2]T = "<<T<<std::endl;

    TransformToPose(T,pose);
    std::cout<<"pose = "<<pose.transpose()<<std::endl;

    Eigen::Matrix<double,7,1> q;
    EulerPoseRPY2QuatPose(pose,q);
    QuatPose2EulerPoseRPY(q,pose);

    std::cout<<"pose2 = "<<pose.transpose()<<std::endl;



        PoseToTransform(pose,T);
    std::cout<<"T2 = "<<T<<std::endl;

    Eigen::Matrix<double,6,6> A;
    TwistTransMatrix(T,A);
    std::cout<<"A = "<<A<<std::endl;

    Eigen::Matrix<double,3,1> pose2d;
    pose2d<<1.0, 2.0, M_PI/4;
    Eigen::Matrix<double,3,3> T2d;
    PoseToTransform(pose2d,T2d);
    std::cout<<"T2d = "<<T2d<<std::endl;

    TransformToPose(T2d,pose2d);
    std::cout<<"pose2d = "<<pose2d<<std::endl;

    Eigen::Matrix<double,3,3> A2d;
    TwistTransMatrix(T2d,A2d);
    std::cout<<"A2d = "<<A2d<<std::endl;







    Eigen::Matrix<double,7,1> pose_base_marker;
    pose_base_marker<<0.406,0.250,0.550,-0.047,0.588,-0.049,0.806;
    Eigen::Matrix<double,6,1> pose_base_marker_rpy;

    QuatPose2EulerPoseRPY(pose_base_marker,pose_base_marker_rpy);
    Eigen::Matrix<double,4,4> T_base_marker;
    PoseToTransform(pose_base_marker_rpy,T_base_marker);
    std::cout<<"pose_base_marker_rpy = "<<pose_base_marker_rpy<<std::endl;
    std::cout<<"T_base_marker = "<<T_base_marker<<std::endl;

    Eigen::Quaterniond quaternion(pose_base_marker(3),pose_base_marker(4),pose_base_marker(5),pose_base_marker(6));


    std::cout<<"R_base_marker = "<<quaternion.toRotationMatrix()<<std::endl;




    Eigen::Matrix<double,7,1> pose_marker_camera;
    pose_marker_camera<<0.201,0.254,0.659,0.073,0.744,-0.655,-0.108;
    Eigen::Matrix<double,6,1> pose_marker_camera_rpy;

    std::cout<<"pose_marker_camera = "<<pose_marker_camera<<std::endl;

    QuatPose2EulerPoseRPY(pose_marker_camera,pose_marker_camera_rpy);
    Eigen::Matrix<double,4,4> T_marker_camera;
    PoseToTransform(pose_marker_camera_rpy,T_marker_camera);

    std::cout<<"pose_marker_camera_rpy = "<<pose_marker_camera_rpy<<std::endl;

    std::cout<<"T_output1 = "<<T_base_marker*T_marker_camera<<std::endl;


    Eigen::Matrix<double,7,1> pose_base_marker1;
    pose_base_marker1<<0.411,-0.022,0.513,-0.047,0.588,-0.049,0.806;
    Eigen::Matrix<double,6,1> pose_base_marker_rpy1;

    QuatPose2EulerPoseRPY(pose_base_marker1,pose_base_marker_rpy1);
    Eigen::Matrix<double,4,4> T_base_marker1;
    PoseToTransform(pose_base_marker_rpy1,T_base_marker1);




    Eigen::Matrix<double,7,1> pose_marker_camera1;
    pose_marker_camera1<<0.196,-0.046,0.668,0.059,0.746,-0.654,-0.115;
    Eigen::Matrix<double,6,1> pose_marker_camera_rpy1;

    QuatPose2EulerPoseRPY(pose_marker_camera1,pose_marker_camera_rpy1);
    Eigen::Matrix<double,4,4> T_marker_camera1;
    PoseToTransform(pose_marker_camera_rpy1,T_marker_camera1);

    std::cout<<"T_output2 = "<<T_base_marker1*T_marker_camera1<<std::endl;



    Eigen::Matrix<double,7,1> pose_base_marker2;
    pose_base_marker2<<0.452,-0.135,0.373,-0.046,0.588,-0.049,0.806;
    Eigen::Matrix<double,6,1> pose_base_marker_rpy2;

    QuatPose2EulerPoseRPY(pose_base_marker2,pose_base_marker_rpy2);
    Eigen::Matrix<double,4,4> T_base_marker2;
    PoseToTransform(pose_base_marker_rpy2,T_base_marker2);




    Eigen::Matrix<double,7,1> pose_marker_camera2;
    pose_marker_camera2<<0.320,-0.181,0.676,0.054,0.747,-0.653,-0.111;
    Eigen::Matrix<double,6,1> pose_marker_camera_rpy2;

    QuatPose2EulerPoseRPY(pose_marker_camera2,pose_marker_camera_rpy2);
    Eigen::Matrix<double,4,4> T_marker_camera2;
    PoseToTransform(pose_marker_camera_rpy2,T_marker_camera2);

    std::cout<<"T_output2 = "<<T_base_marker2*T_marker_camera2<<std::endl;



    Eigen::Matrix<double,7,1> pose_marker_camera3;
    pose_marker_camera3<<0.455173,0.466441,0.394124,0.827531,0.0296718,0.0435978,-0.558937;
    Eigen::Matrix<double,6,1> pose_marker_camera_rpy3;

    QuatPose2EulerPoseRPY(pose_marker_camera3,pose_marker_camera_rpy3);
    Eigen::Matrix<double,4,4> T_marker_camera3;
    PoseToTransform(pose_marker_camera_rpy3,T_marker_camera3);

    std::cout<<"pose_marker_camera_rpy3 = "<<pose_marker_camera_rpy3.transpose()<<std::endl;


    Eigen::Matrix<double,6,1> pose_test_rpy;
    // pose_test_rpy<<0.45518,0.4664,0.394121,0.000316,0.105534,-1.18824;
    pose_test_rpy<<-0.0178478,   0.444668,   0.716594,  -0.775673,   0.581333,   -1.47822;
    // pose_test_rpy<<-0.0178478,   0.444668,   0.716594,  0.0,0.0,   -M_PI_2;

    Eigen::Matrix<double,7,1> temp_qq;
    EulerPoseRPY2QuatPose(pose_test_rpy,temp_qq);
    PoseToTransform(pose_test_rpy,T_base_marker2);
    std::cout<<"T_base_marker2 = "<<T_base_marker2<<std::endl;
    std::cout<<"temp_qq = "<<temp_qq.transpose()<<std::endl;
    QuatPose2EulerPoseRPY(temp_qq,pose_test_rpy);
    std::cout<<"pose_test_rpy = "<<pose_test_rpy.transpose()<<std::endl;


    Eigen::Matrix<double,7,1> q_test2;
    q_test2<<-0.01614,0.44207,0.71465,0.63443,-0.34493,0.25239,-0.64407;
    QuatPose2EulerPoseRPY(q_test2,pose_test_rpy);
    std::cout<<"pose_test_rpy = "<<pose_test_rpy.transpose()<<std::endl;







}

void saveData(std::string fileName, Eigen::MatrixXd  matrix)
{
	//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
	const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

	std::ofstream file(fileName,std::ios::app);
	if (file.is_open())
	{
		file << matrix.format(CSVFormat)<< '\n';
		file.close();
	}
}

