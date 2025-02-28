#include "Spline2DCurve.hpp"


/*1.PCA
2.Load
3.计算曲线参数
4.求导
./devel/lib/diana_driver/Spline2DCurve_node "src/diana_driver/path/Spline 1.txt"

*/
/*
static函数，用于降低任务点的维度。 
输入：std向量 内容为3d点
输出：PCA对应的规划平面
*/
Eigen::Matrix<double,4,4> Spline2DCurve::DimReductionPCA(Eigen::MatrixXd OriMatrix)
{
    //1.去中心化
    int m = OriMatrix.rows();
    int n = OriMatrix.cols();


    //Eigen::Matrix<double,m,n> 
    Eigen::MatrixXd DeCenterMatrix;
    DeCenterMatrix.resize(m,n);

    Eigen::Matrix<double,3,1> MeanValue;


    MeanValue[0] = OriMatrix.row(0).mean();
    MeanValue[1] = OriMatrix.row(1).mean();
    MeanValue[2] = OriMatrix.row(2).mean();

    for(int i=0; i<n;i++)
    {
        DeCenterMatrix.col(i) = OriMatrix.col(i)-MeanValue;
    }

    Eigen::Matrix<double,3,3> DX = 1.0/n*DeCenterMatrix*DeCenterMatrix.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(DX, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    Eigen::MatrixXd C(3, 1);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U = svd.matrixU();  
    C = svd.singularValues();

    std::cout<<"V = "<<V<<std::endl;
    std::cout<<"U = "<<U<<std::endl;
    std::cout<<"C = "<<C<<std::endl;

    // V(0,1) = -V(0,1);
    // V(1,1) = -V(1,1);
    // V(2,1) = -V(2,1);

    assert(isRotationMatirx(V));

    Eigen::Matrix<double,4,4> T;
    T.block(0,0,3,3) = V.transpose();
    T.block(0,3,3,1) =  -V.transpose()*MeanValue;
    Eigen::Matrix<double,1,4> affine_com;
    affine_com<<0,0,0,1;
    T.block(3,0,1,4)  = affine_com;

    

    //std::cout<<"m,n="<<m<<", "<<n<<std::endl;


    return T;
}


void Spline2DCurve::initTimeSegParams()
{
    int k_num = N%N_spline;
    Eigen::MatrixXd Nlist = Eigen::MatrixXd::Ones(1,N_spline)*(N/N_spline);

    //构建Nlist 代表每个曲线对应的点个数（不算开头的点）
    for(int i=0; i<k_num;i++)
    {
        Nlist(i)++;
    }

    
    Nlist_index.resize(1,N_spline);
    //Eigen::Matrix<double,1,N_spline> Nlist_index;
    Nlist_index.setZero();

    //这个列表中第i个元素就是第i段曲线的终端的编号

    for(int i=0;i<N_spline;i++)
    {
        Nlist_index(i) = Nlist.block(0,0,1,i+1).sum(); 

    }
    TimeSeg=1.0/(N-1);
    time_params = (Nlist_index - Eigen::MatrixXd::Ones(1,N_spline))*TimeSeg;
}



void Spline2DCurve::form_H_g_ulbA(Eigen::MatrixXd& A, Eigen::MatrixXd& lb,Eigen::MatrixXd& ub,Eigen::MatrixXd& Qx,bool flag,double Bdbox)
{

    //构造相邻的曲线光滑连续 
    //Eigen::Matrix<double,3*(N_spline-1)+2,6*N_spline> 
    Eigen::MatrixXd Aequ;
    Aequ.resize(3*(N_spline-1)+2,6*N_spline);
    Aequ.setZero();

    Eigen::MatrixXd bequ;
    bequ.resize(3*(N_spline-1)+2,1);
    //Eigen::Matrix<double,3*(N_spline-1)+2,1> bequ;
    bequ.setZero();


    double tt = 0.0;

    for(int i=0;i<N_spline-1;i++)
    {
        tt = (Nlist_index(i)-1)*TimeSeg;
        //曲线连续性 f1 = f2
        std::cout<<"Nlist_index(i) ="<<Nlist_index(i)<<std::endl;
        Aequ.block(i*3,i*6,1,6)<<1,tt,pow(tt,2),pow(tt,3),pow(tt,4),pow(tt,5);
        // std::cout<<"NCC(i) ="<<(i+1)*6<<" "<<6*N_spline<<std::endl;
        Aequ.block(i*3,(i+1)*6,1,6)<<-1,-tt,-pow(tt,2),-pow(tt,3),-pow(tt,4),-pow(tt,5);
        //一阶导数连续性 fDot1 = fDot2
        Aequ.block(i*3+1,i*6,1,6)<<0,1,2*tt,3*pow(tt,2),4*pow(tt,3),5*pow(tt,4);
        Aequ.block(i*3+1,(i+1)*6,1,6)<<-0,-1,-2*tt,-3*pow(tt,2),-4*pow(tt,3),-5*pow(tt,4);
        //二阶导数连续性 fDDot1 = fDDot2
        Aequ.block(i*3+2,i*6,1,6)<<0,0,2,6*tt,12*pow(tt,2),20*pow(tt,3);
        Aequ.block(i*3+2,(i+1)*6,1,6)<<0,0,-2,-6*tt,-12*pow(tt,2),-20*pow(tt,3);

    }

    //增加入角限制
    Aequ(3*(N_spline-1)+1,1) = 1.0;
    Aequ.block(3*(N_spline-1),6*(N_spline-1),1,6)<<0,1,2,3,4,5;
    //X Y 不同
    if(!flag){
        bequ(3*(N_spline-1)+1,0) = (y2D_Task.coeff(0,1)-y2D_Task.coeff(0,0))/TimeSeg;
        bequ(3*(N_spline-1))= (y2D_Task(0,N-1)-y2D_Task(0,N-2))/TimeSeg;
    }
    else{
        bequ(3*(N_spline-1)+1,0) = (y2D_Task.coeff(1,1)-y2D_Task.coeff(1,0))/TimeSeg;
        bequ(3*(N_spline-1))= (y2D_Task(1,N-1)-y2D_Task(1,N-2))/TimeSeg;
    }

    std::cout<<"Aequ = "<<Aequ<<std::endl;
    std::cout<<"bequ = "<<bequ<<std::endl;
    


    //构造不等式
    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(N,6*N_spline);

    std::cout<<"N = "<<N<<std::endl;

    Eigen::MatrixXd boineq_lb = Bdbox*Eigen::MatrixXd::Ones(N,1);
    Eigen::MatrixXd boineq_ub = Bdbox*Eigen::MatrixXd::Ones(N,1);

    int Jindex = 0;
    tt = 0.0;

    for(int i=0; i<N;i++)
    {
        Jindex = 1;
        for(int j=0; j<=N_spline-1;j++)
        {
            if(i<Nlist_index(j))
            {
                Jindex = j;
                break;

            }
        }

        tt = i*TimeSeg;
        std::cout<<"6*Jindex = "<<6*Jindex<<std::endl;
        //TODO
        Aineq.block(i,6*Jindex,1,6)<<1,tt,pow(tt,2),pow(tt,3),pow(tt,4),pow(tt,5);

        //X，Y不同
        if(!flag){

            boineq_lb(i,0) = -Bdbox+y2D_Task(0,i);
            boineq_ub(i,0) = Bdbox+y2D_Task(0,i);
        }
        else{
            boineq_lb(i,0) = -Bdbox+y2D_Task(1,i);
            boineq_ub(i,0) = Bdbox+y2D_Task(1,i);

        }

        
    }



    
    A.resize(3*(N_spline-1)+2+N,6*N_spline);
    A<<Aequ,Aineq;

    std::cout<<"A = "<<A<<std::endl;

    
    lb.resize(3*(N_spline-1)+2+N,1);
    lb<<bequ,boineq_lb;

    std::cout<<"lb = "<<lb<<std::endl;


    
    ub.resize(3*(N_spline-1)+2+N,1);
    ub<<bequ,boineq_ub;

    std::cout<<"ub = "<<ub<<std::endl;





    //构建二次价值函数
    double tstart = 0.0;
    double tend = 0.0;
    double t1 = 0.0;
    double t2 = 0.0;
    double t3 = 0.0;
    double t4 = 0.0;
    double t5 = 0.0;

    Eigen::MatrixXd Q1x = Eigen::MatrixXd::Zero(6*N_spline,6*N_spline);
    for(int i=0;i<N_spline;i++)
    {
        if(i==0)
        {
            tstart = 0;
        }
        else
        {
            tstart = (Nlist_index(i-1)-1)*TimeSeg;
        }

        tend =(Nlist_index(i)-1)*TimeSeg;

        t1 = tend-tstart;
        t2 = pow(tend,2)-pow(tstart,2);
        t3 = pow(tend,3)-pow(tstart,3);
        t4 = pow(tend,4)-pow(tstart,4);
        t5 = pow(tend,5)-pow(tstart,5);

        Q1x.block(6*i+3,6*i+3,3,3)<<36*t1,72*t2,120*t3,72*t2,192*t3,360*t4,120*t3,360*t4,720*t5;


        
    }

    Eigen::MatrixXd Q2x = 0.01*Eigen::MatrixXd::Identity(6*N_spline,6*N_spline);

    Eigen::MatrixXd fx = Eigen::MatrixXd::Zero(6*N_spline,1);

    Qx = Q1x;//+Q2x;
    
    //求解QP问题 using

    std::cout<<Qx<<std::endl;

}

int Spline2DCurve::CalculateOptParams(bool flag, double Bdbox, Eigen::MatrixXd& xopt)
{
    //构造任务矩阵
    //123456
    Eigen::MatrixXd A; 
    Eigen::MatrixXd lb;
    Eigen::MatrixXd ub;
    Eigen::MatrixXd Qx;


    form_H_g_ulbA(A, lb,ub,Qx,flag,Bdbox);
    Eigen::MatrixXd A_coarse;
    A_coarse.conservativeResize(A.rows()*2,A.cols());
    A_coarse<<A,A;


    Eigen::MatrixXd b_coarse;
    b_coarse.conservativeResize(lb.rows()*2,lb.cols());
    b_coarse<<lb,ub;

    Eigen::MatrixXd T_temp = A_coarse.transpose()*A_coarse+0.0001*Eigen::MatrixXd::Identity(A_coarse.cols(), A_coarse.cols());

    Eigen::MatrixXd x_coarse = T_temp.inverse()*A_coarse.transpose()*b_coarse;

    std::cout<<"T_temp = "<<T_temp<<std::endl;
    std::cout<<"A = "<<A<<std::endl;
    std::cout<<"A_coarse = "<<A_coarse<<std::endl;
    std::cout<<"lb = "<<lb<<std::endl;
    std::cout<<"ub = "<<ub<<std::endl;
    std::cout<<"Qx = "<<Qx<<std::endl;
    std::cout<<"x_coarse = "<<x_coarse<<std::endl;


    qpOASES::QProblem example_solver((qpOASES::int_t)Qx.rows(),(qpOASES::int_t)lb.rows());
    qpOASES::Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_TABULAR;
    // options.enableFarBounds = qpOASES::BT_TRUE;
    // options.terminationTolerance = 0.000000000000000000000000000001;
    // options.initialFarBounds = 1e10;
    options.enableEqualities =qpOASES::BT_TRUE;
    //qpOASES::QProblem example_solver(2,1);
	example_solver.setOptions( options );

    qpOASES::int_t nWSR = 1000;

    //qpOASES::real_t* Qxx = Qx.data();
    qpOASES::real_t* Qxx = new double[Qx.size()];
    Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(Qxx, Qx.rows(), Qx.cols()) = Qx;

    qpOASES::real_t* AA = new double[A.size()];
    Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(AA, A.rows(), A.cols()) = A;

    qpOASES::real_t* lbb = new double[lb.size()];
    Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(lbb, lb.rows(), lb.cols()) = lb;
    //qpOASES::real_t* ubb = ub.data();

    qpOASES::real_t* ubb = new double[ub.size()];
    Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>(ubb, ub.rows(), ub.cols()) = ub;

    //std::cout<<"Qxx "<<(Qxx[0])<<std::endl;

    // std::cout<<"AA = "<<AA[0]<<" "<<AA[1]<<" "<<AA[2]<<" "<<AA[3]<<std::endl;

    qpOASES::real_t cpu_time;

    Eigen::MatrixXd g;
    g.resize(6*N_spline,1);
    g.setZero();

    // Options myOptions;
    // myOptions.setToMPC( );
    // myOptions.printLevel = PL_LOW;
    // qp.setOptions( myOptions );



    qpOASES::real_t* xOpt = new double[6*N_spline];
    memset(xOpt, 0, sizeof(xOpt));
    memcpy(xOpt, x_coarse.data(), sizeof(xOpt));


    // example_solver.init( Qx.data(),g.data(),A.data(),nullptr,nullptr,lb.data(),ub.data(), nWSR,nullptr);
	example_solver.init( Qxx,g.data(),AA,nullptr,nullptr,lbb,ubb, nWSR,nullptr,xOpt);
    // qpOASES::real_t* yOptd = new double[6*N_spline+1];
    // memset(yOptd, 0, sizeof(yOptd));
    int ret = example_solver.getPrimalSolution( xOpt );
    // ret = example_solver.getDualSolution( xOpt );
    std::cout<<"example_solver solver = "<<ret<<"  "<<example_solver.isUnbounded()<<" "<<example_solver.isSolved( )<<std::endl;
    // example_solver.getDualSolution(yOptd);
    example_solver.printOptions();
    
    // std::cout<<"xOpt = "<<xOpt[0]<<" "<<xOpt[1]<<" "<<xOpt[2]<<" "<<xOpt[3]<<std::endl;

    //Eigen::MatrixXd xOpt_eigen = Eigen::Map<Eigen::Matrix<double, CC, 1, Eigen::RowMajor>>(xOpt);
    Eigen::Map<Eigen::MatrixXd> xOpt_eigen(xOpt,6*N_spline,1);
    xopt = xOpt_eigen;

    // Eigen::Map<Eigen::MatrixXd> yOpt_eigen(yOptd,6*N_spline+1,1);
    // yopt = yOpt_eigen;
    
    //std::cout<<"output = "<<6*N_spline<<std::endl;
    //example_solver.clear();
    std::cout<<"xOpt_eigen = "<<xOpt_eigen.transpose()<<std::endl;
    std::cout<<"Opt_eigen = "<<A*xopt<<std::endl;


    example_solver.resetCounter();
    //example_solver.reset();
    delete[] Qxx;
	delete[] AA;
	delete[] lbb;
	delete[] ubb;
	delete[] xOpt;
    // delete[] yOptd;
    
    return 0;

}


/*
载入文件 格式为mimics输入 函数初始化
输入：文件路径
输出：类
*/
Spline2DCurve::Spline2DCurve(char* name,int _N_spline)
{

    Eigen::MatrixXd OriMatrix;
    importPathFromFile(name,OriMatrix);
    std::cout<<"OriMatrix:="<<OriMatrix<<std::endl;

    N = OriMatrix.cols();
    std::cout<<"N:="<<N<<std::endl;

    baseFrame = DimReductionPCA(OriMatrix);
    if(N<=2)
    {
        std::cout<<"Error IN POINT NUMBER. The pointnumber should >2"<<std::endl;
        return;
    }


    N_spline = _N_spline;
    initTimeSegParams();

    tt_vector = Eigen::MatrixXd::Zero(1,6*N_spline);

    double BdboxX = 0.0005;
    double BdboxY = 0.0005;

    Eigen::MatrixXd yTemp;
    yTemp.resize(4,N);

    yTemp.block(0,0,3,N)= OriMatrix;
    yTemp.block(3,0,1,N)=Eigen::MatrixXd::Ones(1,N);

    y3D_Task = baseFrame*yTemp;

    std::cout<<"baseFrame = "<<baseFrame<<std::endl;
    std::cout<<"y3D_Task = "<<y3D_Task<<std::endl;
    std::cout<<"yTemp = "<<yTemp<<std::endl;
    
    //y3D_Task.resize(4,N);

    //得到降维后的任务点

    y2D_Task.resize(2,N);
    y2D_Task = y3D_Task.block(0,0,2,N);

    std::cout<<"y2D_Task = "<<y2D_Task<<std::endl;


    int ret = CalculateOptParams(false,BdboxX,xOpt_eigen);
    // std::cout<<"xOpt = "<<xOpt_eigen<<std::endl;

    ret = CalculateOptParams(true,BdboxY,yOpt_eigen);
    // std::cout<<"yOpt = "<<yOpt_eigen<<std::endl;

    tt_vector.resize(1,6*N_spline);


}



void Spline2DCurve::importPathFromFile(char* name,Eigen::MatrixXd& T)
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

int Spline2DCurve::sigma(const double theta, Eigen::Matrix<double,2,1>& y)
{
    if(theta>1|| theta<0)
    {
        return -1;
    }

    int timeIndex = 0;
    for(int i=0; i<N_spline;i++)
    {
        if(theta<=time_params(i))
        {
            timeIndex=i;
            break;
        }
    }

    //Eigen::MatrixXd tt_vector = Eigen::MatrixXd::Zero(1,6*N_spline);
    tt_vector.setZero();
    tt_vector.block(0,6*timeIndex,1,6)<<1,theta,pow(theta,2),pow(theta,3),pow(theta,4),pow(theta,5);

    // std::cout<<"tt_vector = "<<tt_vector<<std::endl;
    
    
    y << tt_vector*xOpt_eigen,tt_vector*yOpt_eigen;
    return 1;

    //Eigen::Matrix<double ><<1,tt,pow(tt,2),pow(tt,3),pow(tt,4),pow(tt,5);
}


int Spline2DCurve::sigmaDot(const double theta, Eigen::Matrix<double,2,1>& yDot)
{
    if(theta>1|| theta<0)
    {
        return -1;
    }

    int timeIndex = 0;
    for(int i=0; i<N_spline;i++)
    {
        if(theta<=time_params(i))
        {
            timeIndex=i;
            break;
        }
    }

    //Eigen::MatrixXd tt_vector = Eigen::MatrixXd::Zero(1,6*N_spline);
    tt_vector.setZero();
    tt_vector.block(0,6*timeIndex,1,6)<<0,1,2*theta,3*pow(theta,2),4*pow(theta,3),5*pow(theta,4);
    
    yDot<< tt_vector*xOpt_eigen , tt_vector*yOpt_eigen;
    return 1;

    //Eigen::Matrix<double ><<1,tt,pow(tt,2),pow(tt,3),pow(tt,4),pow(tt,5);
}

int Spline2DCurve::sigmaDDot(const double theta, Eigen::Matrix<double,2,1>& yDDot)
{
    if(theta>1|| theta<0)
    {
        return -1;
    }

    int timeIndex = 0;
    for(int i=0; i<N_spline;i++)
    {
        if(theta<=time_params(i))
        {
            timeIndex=i;
            break;
        }
    }

    // Eigen::MatrixXd tt_vector = Eigen::MatrixXd::Zero(1,6*N_spline);
    tt_vector.setZero();
    tt_vector.block(0,6*timeIndex,1,6)<<0,0,2,6*theta,12*pow(theta,2),20*pow(theta,3);

    yDDot <<tt_vector*xOpt_eigen,tt_vector*yOpt_eigen;

    return 1;

    //Eigen::Matrix<double ><<1,tt,pow(tt,2),pow(tt,3),pow(tt,4),pow(tt,5);
}

double Spline2DCurve::J(const double theta, const Eigen::Matrix<double,2,1> p)
{
    Eigen::Matrix<double,2,1> pt;
    pt.setZero();
    int ret = sigma(theta, pt);

    double Jo = (p.transpose()-pt.transpose())*(p-pt);
    return Jo; 
}

double Spline2DCurve::JDot2(const double theta, const Eigen::Matrix<double,2,1> p)
{
    Eigen::Matrix<double,2,1> pt;
    pt.setZero();
    int ret = sigma(theta, pt);

    Eigen::Matrix<double,2,1> ptdt;
    ptdt.setZero();
    ret = sigmaDot(theta,ptdt);

    Eigen::MatrixXd JDot = 2.0*(ptdt.transpose()*pt-p.transpose()*ptdt);
    return *(JDot.data());
    //return JDot;
}

double Spline2DCurve::JDDot2(const double theta, const Eigen::Matrix<double,2,1> p)
{
    Eigen::Matrix<double,2,1> pt;
    pt.setZero();
    int ret = sigma(theta, pt);

    Eigen::Matrix<double,2,1> ptdt;
    ptdt.setZero();
    ret = sigmaDot(theta,ptdt);

    Eigen::Matrix<double,2,1> d2ptdt2;
    d2ptdt2.setZero();
    ret = sigmaDDot(theta,d2ptdt2);

    Eigen::MatrixXd Jdd = 2*d2ptdt2.transpose()*pt+2*(ptdt.transpose()*ptdt)-2*p.transpose()*d2ptdt2;

    return *(Jdd.data());
}







int main(int argc, char* argv[])
{
    char* path = argv[1];
    char* theta = argv[2];
    Spline2DCurve instance(path,6);

    Eigen::Matrix<double,2,1> y;

    double th = std::stod(theta);

    instance.sigma(th,y);
    std::cout<<"y2D_Task = "<<instance.y2D_Task<<std::endl;
    std::cout<<"xOpt_eigen = "<<instance.xOpt_eigen.transpose()<<std::endl;
    std::cout<<"yOpt_eigen = "<<instance.yOpt_eigen.transpose()<<std::endl;
    std::cout<<"y = "<<y<<std::endl;
    

    return 0;
    
}

