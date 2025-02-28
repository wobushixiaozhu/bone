#include "Controllers.hpp"



ControllersSISO::ControllersSISO()
{

}

ControllersSISO::~ControllersSISO()
{
    
}

double ControllersSISO::Update(double input)
{
    return 0;
}
double ControllersSISO::Update(double r, double y)
{
    return 0.0;
}


RobustForceControl::RobustForceControl()
{

    b0 = 0.1165;
    b1 = -0.03617;
    b2 = -0.08041;
    b3 = 0.0001052;
    // b4 = -0.8368;

    a1 = -2.632;
    a2 =  2.367;
    a3 =  -0.7345;
    a4 =  3.378e-6;

    x_0 = 0.0;
    x_1 = 0.0;
    x_2 = 0.0;
    x_3 = 0.0;
    x_4 = 0.0;

    o_1 = 0.0;
    o_2 = 0.0;
    o_3 = 0.0;
    o_4 = 0.0;

    r_last = 0.0;
    status = 0;

}

RobustForceControl::~RobustForceControl()
{
    
}

double RobustForceControl::Update(double input)
{
    // x_4 = x_3;
    x_3 = x_2;
    x_2 = x_1;
    x_1 = x_0;
    x_0 = input;


    double o=-a1*o_1-a2*o_2-a3*o_3-a4*o_4+  x_3*b3+ x_2*b2 +x_1*b1+ x_0*b0;
    
    o_4 = o_3;
    o_3 = o_2;
    o_2 = o_1;
    o_1 = o;

    return o;
}

double RobustForceControl::Update(double r, double y)
{
    double x = r-y;
    double o = 0.0;

    if(50.0*r<-0.2 && 50.0*r_last>=-0.2)
    {
        o=Update(x);
        status = 1;
    }
    else if(50.0*r>-0.2 && 50.0*r_last<=-0.2)
    {
        o = x;
        parasetZero();
        status = 0;
    }
    else
    {
        switch (status)
        {
        case 1:
            o=Update(x);
            break;
        
        default:
            o = x;
            break;
        }
    }

    r_last = r;
    // // x_4 = x_3;
    // x_3 = x_2;
    // x_2 = x_1;
    // x_1 = x_0;
    // x_0 = input;


    // double o=-a1*o_1-a2*o_2-a3*o_3-a4*o_4+  x_3*b3+ x_2*b2 +x_1*b1+ x_0*b0;
    
    // o_4 = o_3;
    // o_3 = o_2;
    // o_2 = o_1;
    // o_1 = o;

    return o;
}

int RobustForceControl::parasetZero()
{
    // x_4 = x_3;
    x_3 = 0.0;
    x_2 = 0.0;
    x_1 = 0.0;
    x_0 = 0.0;


    // double o=-a1*o_1-a2*o_2-a3*o_3-a4*o_4+  x_3*b3+ x_2*b2 +x_1*b1+ x_0*b0;
    
    o_4 = 0.0;
    o_3 = 0.0;
    o_2 = 0.0;
    o_1 = 0.0;

    return 0;
}




ARDCMotionControl::ARDCMotionControl()
{
    tdfilter = new TD2Order(0.01,1000.0,0.01); //Ts , r, h

    // parameters for non-linear PD controller
    alpha1 = 0.5;
    alpha2 = 1.25;

    delta = 0.01;

    beta1 = 3.133; //P
    beta2 = 3.720; //D

    // fal1 = 0.0;
    // fal2 = 0.0;


    // parameters for ESO Observer
    O_beta1 = 1.0;
    O_beta2 = 0.2;
    O_beta3 = 0.01;

    O_b = 5.0;
    O_h = 0.01;
    O_alpha1 = 0.5;
    O_alpha2 = 0.25;
    O_delta = 0.01;

    z1_k_1 = 0.0;
    z2_k_1 = 0.0;
    z3_k_1 = 0.0;

    //updata memories
    u_last = 0.0;
     
}

double ARDCMotionControl::sign(double e)
{
    double y = 0.0;
    if(e>0)
    {
        y = 1.0;
    }
    else if(e<0)
    {
        y = -1.0;
    }
    return y;
}

double ARDCMotionControl::fal(double e, double delta, double alpha)
{
    double fal = 0.0;
    if (std::abs(e)<=delta)
    {
        fal = e/(std::pow(delta,(1-alpha)));
    }
    else
    {
        fal = std::pow(std::abs(e),alpha)*sign(e);
    }
    
    return fal;
}

int ARDCMotionControl::ESO(double yk,double uk_1, double& z1_k, double& z2_k, double& z3_k)
{
    //z1
    double e1 = z1_k_1-yk;
    z1_k = z1_k_1 + O_h*z2_k_1 -O_beta1*e1;
    z1_k_1 = z1_k;

    //z2
    double fal1 = fal(e1,O_delta,O_alpha1);
    z2_k=z2_k_1+O_h*(z3_k_1+O_b*uk_1)-O_beta2*fal1;
    z2_k_1=z2_k;

    //z3
    double fal2 = fal(e1,O_delta,O_alpha2);
    z3_k=z3_k_1-O_h*O_beta3*fal2;
    z3_k_1=z3_k;

    return 1;
}

double ARDCMotionControl::NonLinearPD(double e1,double e2)
{

    //
    double fal1 = fal(e1,delta,alpha1);
    double fal2 = fal(e2,delta,alpha2);

    double uk = beta1*fal1+ beta2*fal2;

    return uk;
}

double ARDCMotionControl::Update(double r, double y)
{
    double x1 = 0.0;
    double x2 = 0.0;
    double z1 = 0.0;
    double z2 = 0.0;
    double z3 = 0.0;

    tdfilter->update(r,x1,x2);

    ESO(y,u_last,z1,z2,z3);

    // Eigen::Matrix<double,3,1> zz;
    // zz<<z1,z2,z3;

    // Eigen::Matrix<double,2,1> xx;
    // xx<<x1,x2;

    // saveData(std::string("/home/agr/catkin_ws/zz.csv"), zz.transpose());
    // saveData(std::string("/home/agr/catkin_ws/xx.csv"), xx.transpose());

    double e1 = x1 - z1;
    double e2 = x2 - z2;

    double u = NonLinearPD(e1,e2)-z3;

    u_last = u;
    return u;
}


