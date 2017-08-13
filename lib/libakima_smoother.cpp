/*
 * Copyright (c) 2017 Candido Otero Moreira (omcandido@uvigo.es)
 *
 * Based on the H. Akima spline interpolation approach [Akima, October 1970]
 * (Equations are referred to according to his paper)
 */

#include <libakima_smoother.h>

akimaSmoother::akimaSmoother(std::vector<geometry_msgs::Point> &rawPoints)
{
    //Initializes an akimaPoints vector with the X and Y values of the rawPath.
    //Then computes all parameters at each given point
    for (geometry_msgs::Point iterPoint: rawPoints)
    {
        akimaPoint AkimaPoint(iterPoint.x,iterPoint.y);
        Points.push_back(AkimaPoint);
    }
}

void akimaSmoother::computeSlope()
{
    for (size_t i=0;i<getSize()-1;i++)
    {
        Points[i].updateSlope(b(i)/a(i)); //eq [14]
    }
        Points.back().updateSlope(Points[getSize()-1].getParams().m);
}

void akimaSmoother::computeGradient()
{
    double t;
    for (size_t i=2;i<getSize()-2;i++)
    {
        if ((Points.at(i-2).getParams().m == Points.at(i-1).getParams().m and Points.at(i-1).getParams().m == Points.at(i).getParams().m)
            or (Points.at(i-1).getParams().m == Points.at(i).getParams().m and Points.at(i).getParams().m == Points.at(i+1).getParams().m))
        {
            t= Points.at(i-1).getParams().m;
        }
        else
        {
            double w2,w3;
            /*
            if((Points.at(i-1).getParams().m == Points.at(i+1).getParams().m and
                Points.at(i).getParams().m != Points.at(i-2).getParams().m and
                Points.at(i+1).getParams().m != Points.at(i).getParams().m)
                or
               (Points.at(i).getParams().m == Points.at(i-2).getParams().m and
                Points.at(i-1).getParams().m != Points.at(i-2).getParams().m and
                Points.at(i+1).getParams().m != Points.at(i-1).getParams().m))
            {
                w2 = fabs(S(i,i+1));
                w3 = fabs(S(i-2,i-1));
            }
            else
            {
                w2 = std::sqrt(fabs(S(i-2,i)*S(i,i+1)));
                w3 = std::sqrt(fabs(S(i-2,i-1)*S(i-1,i+1)));
            }
            */

            w2 = fabs(S(i,i+1));
            w3 = fabs(S(i-2,i-1));

            w2 = std::sqrt(fabs(S(i-2,i)*S(i,i+1)));
            w3 = std::sqrt(fabs(S(i-2,i-1)*S(i-1,i+1)));

            t = (   (w2 * b(i-1) + w3 * b(i))
                  / (w2 * a(i-1) + w3 * a(i)) ); //eqs [31][39][40]
        }
        Points[i].updateGradient(t);
        //std::cout << "i: "<< i << " t: "<< t<<std::endl;
    }
}

void akimaSmoother::computeParams()
{
    //Note: the vector Points is assumed to already have 2 additional points at the begining and at the end

    computeSlope();
    computeGradient();

    double p[4],q[4];
    double r;
    for (size_t i = 2;i<getSize()-2;i++)
    {
        r = std::sqrt(a(i)*a(i)+b(i)*b(i));

        p[0] = Points[i].getX(); //eq [57]
        p[1] = r*cosTh(i);  //eq [58]
        p[2] = 3*a(i) - r*( cosTh(i+1) + 2*cosTh(i) ); //eq [59]
        p[3] = -2*a(i) + r*( cosTh(i+1) + cosTh(i) ); //eq [60]

        q[0] = Points[i].getY();  //eq [60]
        q[1] = r*sinTh(i);  //eq [62]
        q[2] = 3*b(i) - r*( sinTh(i+1) + 2*sinTh(i) );  //eq [63]
        q[3] = -2*b(i) + r*( sinTh(i+1) + sinTh(i) );  //eq [64]

        Points[i].updateCoefs(p,q);
    }
}

void akimaSmoother::smoothOpenPath()
{
    addPointsBack();
    addPointsFront();
    computeParams();
}

void akimaSmoother::addPointsBack()
{
    int N = getSize()-1;

    double x4 = 2*a(N-1) - a(N-2) + Points.back().getX(); //eq [67]
    double x5 = 2*(x4-Points.back().getX()) - a(N-1) + x4; //eq [67]
    double y4 = 2*b(N-1) - b(N-2) + Points.back().getY();//eq [68]
    double y5 = 2*(y4-Points.back().getY()) - b(N-1) + y4;//eq [68]

    akimaPoint p4(x4,y4);
    Points.push_back(p4);
    akimaPoint p5(x5,y5);
    Points.push_back(p5);
}

void akimaSmoother::addPointsFront()
{
    double x2 = a(1) - 2*a(0) + Points.front().getX();//eq [67]
    double x1 = a(0) - 2*(Points.front().getX()-x2) + x2;//eq [67]
    double y2 = b(1) - 2*b(0) + Points.front().getY();//eq [68]
    double y1 = b(0) - 2*(Points.front().getY() - y2) + y2;//eq [68]

    akimaPoint p2(x2,y2);
    Points.insert(Points.begin(),p2);
    akimaPoint p1(x1,y1);
    Points.insert(Points.begin(),p1);
}
