#include "TD2Order.hpp"


int sign(int x)
{
	if(x>0)
		return 1;
	else if(x==0)//一定要是==
		return 0;
	else
		return -1;

}

double fhan(double x1, double x2, double u,
                double r,double h)
{
    double d = r*h;
    double d0 = d*h;
    double y = x1-u+h*x2;
    double a0=std::sqrt(d*d+8*r*std::abs(y));

    double a,f;

    if (std::abs(y)<=d0)
    {
        a=x2+y/h;
    }
    else
    {
        a=x2+0.5*(a0-d)*sign(y);
    }


    if(std::abs(a)<=d)
    {
        f=-r*a/d;
    }
    else
    {
        f=-r*sign(a);
    }

    return f;

}




TD2Order::TD2Order(double para_Ts, double para_r, double para_h)
{
    Ts = para_Ts;
    r = para_r;
    h = para_h;

    x_1 = 0.0;
    x_2 = 0.0;
}

int TD2Order::update(double u, double& x1, double& x2)
{
    double x1k = x_1;
    double x2k = x_2;
    x_1 = x1k+Ts* x2k;
    x_2 = x2k+Ts*fhan(x1k,x2k,u,r,h);
    // std::cout<<"x_2= "<<x_2<<std::endl;
    // std::cout<<"Ts= "<<Ts<<std::endl;
    // std::cout<<"fhan(x1k,x2k,u,r,h)= "<<fhan(x1k,x2k,u,r,h)<<std::endl;
    x1 = x_1;
    x2 = x_2;
    return 1;
}



int main()
{
    Eigen::Matrix<double,6,1> r;
    r<< 1.0,2.0,3.0,4.0,5.0,6.0;

    TD2Order instance(0.001,1000.0,0.05);

    double x1,xdot;

    for(int i =0; i<2000;i++)
    {
        instance.update(r(0),x1,xdot);
        std::cout<<"x1 = "<<x1<<std::endl;
    }


    return 1;

    
}
