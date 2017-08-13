/*
 * Copyright (c) 2017 Candido Otero Moreira (omcandido@uvigo.es)
 *
 * Based on the H. Akima spline interpolation approach [Akima, October 1970]
 * (Equations are referred to according to his paper)
 */

#ifndef LIBAKIMA_SMOOTHER_H
#define LIBAKIMA_SMOOTHER_H

#include <vector>
#include <nav_msgs/Path.h>

struct akimaParams
{
    double m;
    double t;
    double p[4];
    double q[4];
};

class akimaPoint
{
private:
    double x;
    double y;
    akimaParams params;
public:
    akimaPoint(double _x, double _y):
        x(_x),
        y(_y)
    {}
    ~akimaPoint(){}
    double getX() const {return x;}
    double getY() const {return y;}
    akimaParams getParams() const {return params;}
    void updateSlope(const double _m) {params.m=_m;}
    void updateGradient(const double _t) {params.t=_t;}
    void updateCoefs(const double _p[], const double _q[4])
    {
        std::copy(_p,_p+4,params.p);
        std::copy(_q,_q+4,params.q);
    }
};

class akimaSmoother
{
private:
    std::vector<akimaPoint> Points;
    void computeSlope(); //m
    void computeGradient(); //t
    void addPointsFront();
    void addPointsBack();
    void computeParams();

public:
    akimaSmoother(std::vector<geometry_msgs::Point> &rawPoints);
    ~akimaSmoother(){}
    void smoothOpenPath();
    size_t getSize() const {return Points.size();}
    double a(const int i) const {return (Points[i+1].getX()-Points[i].getX());} //eq [12]
    double b(const int i) const {return (Points[i+1].getY()-Points[i].getY());} //eq [13]
    double S(const int i, const int j) const
    {
        assert(i!=j && "ERROR: i=j when computing S(i,j)");
        return a(i)*b(j)-a(j)*b(i); //eq [28]
    }
    double cosTh(const int i)const
    {
        double a0 = fabs(S(i,i+1)) * a(i-1) + fabs(S(i-2,i-1)) * a(i);//eq [48]
        double b0 = fabs(S(i,i+1)) * b(i-1) + fabs(S(i-2,i-1)) * b(i);//eq [49]

        if (a0!=0 and b0!=0) return a0/std::sqrt(a0*a0+b0*b0); //eq [46]

        double th= atan2(b(i),a(i));
        return cos(th);


    }
    double sinTh(const int i)const
    {
        double a0 = fabs(S(i,i+1)) * a(i-1) + fabs(S(i-2,i-1)) * a(i);//eq [48]
        double b0 = fabs(S(i,i+1)) * b(i-1) + fabs(S(i-2,i-1)) * b(i);//eq [49]

        if (a0!=0 and b0!=0) return b0/std::sqrt(a0*a0+b0*b0); //eq [47]

        double th = atan2(b(i),a(i));
        return sin(th);

    }
    std::vector<akimaPoint> * getPoints(){return &Points;}
    double segmentLength(const int i )const
    {
        return std::sqrt( (Points.at(i+1).getX()-Points.at(i).getX())*(Points.at(i+1).getX()-Points.at(i).getX())
                  +(Points.at(i+1).getY()-Points.at(i).getY())*(Points.at(i+1).getY()-Points.at(i).getY()) ); //Euclidean distance
    }
};

#endif //LIBAKIMA_SMOOTHER_H
