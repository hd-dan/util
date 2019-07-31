#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <math.h>
#include <cmath> //abs
#include <stdlib.h>
//#include <stdio.h> //printf
#include <iostream> //cout
#include <iomanip> //cout set width
#include <ctime>
#include <chrono>

#include <boost/function.hpp>


template <class num>
num deg2rad(num x){
    return M_PI/180.0*x;
}

template <class num>
num rad2deg(num x){
    return 180.0/M_PI*x;
}

template <class num>
std::vector<num> deg2rad(std::vector<num> x){
    for (unsigned int i=0;i<x.size();i++){
        x.at(i)*= M_PI/180.0;
    }
    return x;
}

template <class num>
std::vector<num> rad2deg(std::vector<num> x){
    for (unsigned int i=0;i<x.size();i++){
        x.at(i)*= 180.0/M_PI;
    }
    return x;
}

template <class num>
int sign(num x, double thres=1e-10){
    return(x<thres)? -1: 1;
}

template <class num>
num roundZeroThres(num x){
    return (fabs(x)<=1e-6)?0:x;
}

template <class num>
std::vector<double> rotm2quat(std::vector<std::vector<num> > R){
    std::vector<double> quat;
    double r11,r12,r13, r21,r22,r23, r31,r32,r33, s,vx,vy,vz;
    r11= R.at(0).at(0); r12= R.at(0).at(1); r13= R.at(0).at(2);
    r21= R.at(1).at(0); r22= R.at(1).at(1); r23= R.at(1).at(2);
    r31= R.at(2).at(0); r32= R.at(2).at(1); r33= R.at(2).at(2);

    double qs2= roundZeroThres(r11+r22+r33+1);
    double qx2= roundZeroThres(r11-r22-r33+1);
    double qy2= roundZeroThres(-r11+r22-r33+1);
    double qz2= roundZeroThres(-r11-r22+r33+1);

    s= 0.5*sqrt(qs2);
    vx= -0.5*sign(r23-r32)* sqrt(qx2);
    vy= -0.5*sign(r31-r13)* sqrt(qy2);
    vz= -0.5*sign(r12-r21)* sqrt(qz2);

    quat.push_back(vx);
    quat.push_back(vy);
    quat.push_back(vz);
    quat.push_back(s);

    return quat;
}

template <class num>
std::vector<std::vector<double> > quat2rotm(std::vector<num> quat){
    std::vector<std::vector<double> > R(3,std::vector<double>(3,0));
    double s,vx,vy,vz;
    vx= quat.at(0);
    vy= quat.at(1);
    vz= quat.at(2);
    s= quat.at(3);

    R.at(0).at(0)= 2*s*s-1 + 2*vx*vx;
    R.at(0).at(1)= 2*vx*vy + -2*s*vz;
    R.at(0).at(2)= 2*vx*vz + 2*s*vy;

    R.at(1).at(0)= 2*vy*vx + 2*s*vz;
    R.at(1).at(1)= 2*s*s-1 + 2*vy*vy;
    R.at(1).at(2)= 2*vy*vz + -2*s*vx;

    R.at(2).at(0)= 2*vz*vx + -2*s*vy;
    R.at(2).at(1)= 2*vz*vy + 2*s*vx;
    R.at(2).at(2)= 2*s*s-1 + 2*vz*vz;

    return R;
}

template <class num>
std::vector<num> axisAngle2quat(std::vector<num>axisAngle){
    std::vector<num> axis(axisAngle.begin(),axisAngle.begin()+3);
    double l2norm= 0;
    for (unsigned int i=0;i<3;i++){
        l2norm+= axis.at(i)*axis.at(i);
    }
    l2norm=sqrt(l2norm);
    if (fabs(l2norm-1)>1e-4){
//        printf("Axis is not unit vector. Converting to unit vector now.\n");
        axis= 1./l2norm*axis;
    }

    std::vector<num> quat(4,0);
    for (unsigned int i=0;i<3;i++){
        quat.at(i)= axis.at(i)*sin(axisAngle.at(3)*0.5);
    }
    quat.at(3)= cos(axisAngle.at(3)*0.5);
    return quat;
}

template <class num>
std::vector<num> quat2axisAngle(std::vector<num>quat){
    std::vector<num> axisAngle(4,0);
    double sq= 0;
    for(unsigned int i=0;i<3;i++){
        sq+=quat.at(i)*quat.at(i);
    }
    sq= sqrt(sq);

    for(unsigned int i=0;i<3;i++){
        axisAngle.at(i)=quat.at(i)/sq;
    }
    axisAngle.at(3)= 2*atan2(sq,quat.at(3));
    return axisAngle;
}

template <class num>
std::vector<std::vector<num> > axisAngle2rotm(std::vector<num> axisAngle){
    return quat2rotm(axisAngle2quat(axisAngle));
}

template <class num>
std::vector<num> rotm2axisAngle(std::vector<std::vector<num> > R){
    return quat2axisAngle(rotm2quat(R));
}

template <class num>
std::vector<num> quatmul(std::vector<num> q1, std::vector<num> q2){
    std::vector<num> q12;
    num x1,y1,z1,s1, x2,y2,z2,s2, x12,y12,z12,s12;
    x1= q1.at(0);
    y1= q1.at(1);
    z1= q1.at(2);
    s1= q1.at(3);

    x2= q2.at(0);
    y2= q2.at(1);
    z2= q2.at(2);
    s2= q2.at(3);

    x12= x1*s2 + s1*x2 + y1*z2 - z1*y2;
    y12= s1*y2 - x1*z2 + y1*s2 + z1*x2;
    z12= s1*z2 + x1*y2 - y1*x2 + z1*s2;
    s12= s1*s2 - x1*x2 - y1*y2 - z1*z2;

    q12.push_back(x12);
    q12.push_back(y12);
    q12.push_back(z12);
    q12.push_back(s12);

    return q12;
}

template <class num>
std::vector<num> quatInv(std::vector<num> q){
    q.at(0)*=-1;
    q.at(1)*=-1;
    q.at(2)*=-1;
    return q;
}

template <class num>
void print_num(num x){
    printf("[ ");
    std::cout<< std::setw(12) <<  std::fixed << std::setprecision(7) << x <<", ";
    printf(" ]\n");
}

template <class num>
void print_num(std::string name, num x){
    printf("%s: ",name.c_str());
    print_num(x);
}

template <class num>
void print_vector(std::vector<num> x){
    printf("[ ");
    for (unsigned int i=0;i<x.size();i++){
        std::cout<< std::setw(12) <<  std::fixed << std::setprecision(7) << x.at(i) <<", ";
    }
    printf(" ]\n");
}

template <class num>
void print_vector(std::string name, std::vector<num> x){
    printf("%s: ",name.c_str());
    print_vector(x);
}

template <class num>
void print_matrix(std::vector<std::vector<num> > x){
    printf("[");
    for (unsigned long i=0;i<x.size();i++){
        print_vector(x.at(i));
    }
    printf("]\n");
}

template <class num>
void print_matrix(std::string name, std::vector<std::vector<num> > x){
    printf("%s: \n",name.c_str());
    print_matrix(x);
}

template <class num>
num elapsedTime(std::chrono::high_resolution_clock::time_point t0, std::chrono::high_resolution_clock::time_point t1){
    return std::chrono::duration_cast<std::chrono::duration<num> >(t1-t0).count();
}

template <class num>
std::vector<std::vector<num> > makeMat(int m, int n, num x){
    return std::vector<std::vector<num> >(m, std::vector<num>(n,x));
}


template <class mat>
std::vector<std::vector<mat> > transpose(std::vector<std::vector<mat> > A){
    unsigned long am,an;
    am= A.size();
    if (am==0){
        return A;
    }
    an= A.at(am-1).size();
    std::vector<std::vector<mat> > AT(an, std::vector<mat>(am) );
    for(unsigned long i=0;i<am;i++){
        for(unsigned long j=0;j<A.at(i).size();j++){
            AT.at(j).at(i)= A.at(i).at(j);
        }
    }
    return AT;
}

template <class mat>
std::vector<std::vector<mat> > matmul(std::vector<std::vector<mat> > A, std::vector<std::vector<mat> > B){
    unsigned long am,an, bm,bn;
    am= A.size(); an= A.at(0).size();
    bm= B.size(); bn= B.at(0).size();

    if (an!=bm){
        printf("A: %ldx%ld & B: %ldx%ld cannot multiply\n",am,an,bm,bn);
        exit(1);
    }

    std::vector<std::vector<mat> > M(am, std::vector<mat>(bn,0) );
    for(unsigned long i=0;i<am;i++){
        for(unsigned long j=0;j<bn;j++){
            for(unsigned long k=0;k<bm;k++){
                M[i][j]+= A[i][k]*B[k][j];
            }
        }
    }
    return M;
}

template <class num>
std::vector<num> matmul(std::vector<std::vector<num> > A, std::vector<num> b){
    unsigned int am, an, bm;
    am= A.size();
    an= A.at(0).size();
    bm= b.size();

    if (an!=bm){
        printf("A: %dx%d & b: %dx1 cannot multiply\n",am,an,bm);
        exit(1);
    }

    std::vector<num> y(am,0);
    for (unsigned int i=0;i<am;i++){
        for (unsigned int j=0;j<an;j++){
            y.at(i)+= A.at(i).at(j)*b.at(j);
        }
    }
    return y;
}

template <class num>
std::vector<num> matmul(std::vector<num> u, std::vector<std::vector<num> > A){
    unsigned int un,am,an;
    un= u.size();
    am= A.size(); an= A.at(0).size();
    if (un!=am){
        printf("x: 1x%d & A: %dx%d cannot multiply\n",un,am,an);
        exit(1);
    }

    std::vector<num> x(A.at(0).size(),0);
    for (unsigned i=0;i<x.size();i++){
        for (unsigned j=0;j<A.size();j++){
            x.at(i)+= u.at(j)*A.at(j).at(i);
        }
    }
    return x;
}

template <class num>
std::vector<std::vector<num> > outermul(std::vector<num> x, std::vector<num> y){
    unsigned int xn,ym;
    xn= x.size();
    ym= y.size();
    if (xn!=ym){
        printf("x: 1x%d & y: %dx1 cannot multiply\n",xn,ym);
        exit(1);
    }

    std::vector<std::vector<num> > A(x.size(),std::vector<num>(y.size(),0));
    for (unsigned i=0;i<x.size();i++){
        for(unsigned j=0;j<y.size();j++){
            A.at(i).at(j)= x.at(i)*y.at(j);
        }
    }
    return A;
}

template <class num>
std::vector<num> extractVect(std::vector<std::vector<num> > A, int m0, int m1, int n0, int n1){
    std::vector<num> x;
    if (m0<0)
        m0+= A.size();
    if (m1 <0)
        m1+= A.size();
    if (n0<0)
        n0+= A.at(0).size();
    if (n1 <0)
        n1+= A.at(0).size();
    m0= (m0<0)?0:m0;
    m1= (m1<0)?0:m1;
    n0= (n0<0)?0:n0;
    n1= (n1<0)?0:n1;

    for (unsigned int i=unsigned(m0);i<unsigned(m1)+1;i++){
        for (unsigned int j=unsigned(n0);j<unsigned(n1)+1;j++){
            x.push_back(A.at(i).at(j));
        }
    }
    return x;
}

template <class num>
std::vector<num> extractVect(std::vector<num> x, int m0, int m1){
    std::vector<num> y;
    if (m0<0)
        m0+= x.size();
    if (m1<0)
        m1+= x.size();
    m0= (m0<0)?0:m0;
    m1= (m1<0)?0:m1;

    for (unsigned int i=unsigned(m0);i<unsigned(m1)+1;i++){
        y.push_back(x.at(i));
    }
    return y;
}

template <class num>
std::vector<std::vector<num> > extractMat(std::vector<std::vector<num> > A, int m0, int m1, int n0, int n1){
    std::vector<std::vector<num> > B;
    if (m0<0)
        m0+= A.size();
    if (m1 <0)
        m1+= A.size();
    m0= (m0<0)?0:m0;
    m1= (m1<0)?0:m1;

    if (n0==0 && n1==-1){
        for(unsigned long i=unsigned(m0);i<unsigned(m1)+1;i++){
            B.push_back(A.at(i));
        }
        return B;
    }

    if (n0<0)
        n0+= A.at(0).size();
    if (n1 <0)
        n1+= A.at(0).size();
    n0= (n0<0)?0:n0;
    n1= (n1<0)?0:n1;

    for (unsigned long i=unsigned(m0);i<unsigned(m1)+1;i++){
        std::vector<num> x;
        for (unsigned long j=unsigned(n0);j<unsigned(n1)+1;j++){
            x.push_back(A.at(i).at(j));
        }
        B.push_back(x);
    }
    return B;
}

template <class num>
std::vector<std::vector<num> > putInMat(std::vector<std::vector<num> >A, std::vector<num> x,
                                        unsigned int m0, unsigned int m1, unsigned int n0, unsigned n1){
    unsigned int am,an,xmn;
    am= A.size(); an= A.at(0).size();
    xmn= x.size();
    if (m0>am || m1>am || n0>an || n1>an){
        printf("[%d:%d,%d:%d] exceed A %dx%d\n",m0,m1,n0,n1,am,an);
        exit(1);
    }
    if (xmn!= (m1-m0)+(n1-n0)+1){
        printf("[%d:%d,%d:%d] not equal to dim of x:%d\n",m0,m1,n0,n1,xmn);
        exit(1);
    }

    for (unsigned int i=m0,k=0;i<m1+1;i++){
        for (unsigned int j=n0;j<n1+1;j++,k++){
            A.at(i).at(j)= x.at(k);
        }
    }
    return A;
}

template <class num>
std::vector<std::vector<num> > putInMat(std::vector<std::vector<num> >A, std::vector<std::vector<num> > B,
                                        unsigned int m0, unsigned int m1, unsigned int n0, unsigned n1){
    unsigned int am,an,bm,bn;
    am= A.size(); an= A.at(0).size();
    bm= B.size(); bn= B.at(0).size();
    if (m0>am || m1>am || n0>an || n1>an){
        printf("[%d:%d,%d:%d] exceed A %dx%d\n",m0,m1,n0,n1,am,an);
        exit(1);
    }
    if (bm!= m1-m0+1 || bn!= n1-n0+1){
        printf("[%d:%d,%d:%d] not equal to dim of A:%dx%d\n",m0,m1,n0,n1,bm,bn);
        exit(1);
    }

    for (unsigned int i=m0,k=0;i<m1+1;i++,k++){
        for (unsigned int j=n0,l=0;j<n1+1;j++,l++){
            A.at(i).at(j)= B.at(k).at(l);
        }
    }
    return A;
}

template <class num>
std::vector<num> appendVect(std::vector<num> x, std::vector<num> y){
    for (unsigned i=0;i<y.size();i++){
        x.push_back(y.at(i));
    }
    return x;
}

template <class num>
std::vector<num> appendVect(std::vector<num> x, num y){
    x.push_back(y);
    return x;
}

template <class num>
std::vector<std::vector<num> > appendMat(std::vector<std::vector<num> > A, std::vector<std::vector<num> > B, int dir= 0){
    unsigned int am,an,bm,bn;
    am= A.size(); an= A.at(0).size();
    bm= B.size(); bn= B.at(0).size();
    if (dir==0){
        if (an!=bn){
            printf("A: %dx%d & B: %dx%d cannot stack vertically\n",am,an,bm,bn);
            exit(1);
        }
        for (unsigned i=0;i<B.size();i++){
            A.push_back(B.at(i));
        }
    }else{
        if (am!=bm){
            printf("A: %dx%d & B: %dx%d cannot stack vertically\n",am,an,bm,bn);
            exit(1);
        }
        std::vector<std::vector<num> > C;
        for (unsigned i=0;i<B.size();i++){
            C.push_back( appendVect(A.at(i),B.at(i)) );
        }
        A= C;
    }

    return A;
}

template <class num>
std::vector<num> add(std::vector<num> a, std::vector<num> b, double alpha=1, double beta=1){
    unsigned int am= a.size();
    unsigned int bm= b.size();
    if(am!=bm){
        printf("a: %dx1 & b: %dx1 cannot be added\n",am,bm);
        exit(1);
    }
    std::vector<num> c(a.size(),0);
    for (unsigned int i=0;i<a.size();i++){
        c.at(i)= alpha*a.at(i) + beta*b.at(i);
    }
    return c;
}

template <class num>
std::vector<num> minus(std::vector<num> a, std::vector<num> b, double alpha=1, double beta=-1){
    return add(a,b,alpha,beta);
}

template <class num>
std::vector<std::vector<num> >add(std::vector<std::vector<num> >A, std::vector<std::vector<num> > B, double alpha=1, double beta=1){
    unsigned int am,an,bm,bn;
    am= A.size(); an= A.at(0).size();
    bm= B.size(); bn= B.at(0).size();
    if(am!=bm || an!=bn){
        printf("A: %dx%d & B: %dx%d cannot be added\n",am,an,bm,bn);
        exit(1);
    }
    std::vector<std::vector<num> > C;
    for (unsigned i=0;i<A.size();i++){
        C.push_back( add(A.at(i),B.at(i), alpha,beta) );
    }
    return C;
}

template <class num>
std::vector<std::vector<num> >minus(std::vector<std::vector<num> >A, std::vector<std::vector<num> > B, double alpha=1, double beta=-1){
    return add(A,B,alpha,beta);
}

template <class num>
std::vector<num> scal(std::vector<num> a, double alpha){
    std::vector<num> c(a.size(),0);
    for (unsigned int i=0;i<a.size();i++){
        c.at(i)= alpha*a.at(i);
    }
    return c;
}

template <class num>
std::vector<std::vector<num> > scal(std::vector<std::vector<num> > A, double alpha){
    std::vector<std::vector<num> > aA;
    for (unsigned i=0;i<A.size();i++){
        aA.push_back(scal(A.at(i),alpha));
    }
    return aA;
}

template <class num>
std::vector<num> operator+(std::vector<num> a, std::vector<num> b){
    return add(a,b);
}
template <class num>
std::vector<std::vector<num> > operator+(std::vector<std::vector<num> >A, std::vector<std::vector<num> > B){
    return add(A,B);
}

template <class num>
std::vector<num> operator-(std::vector<num> a, std::vector<num> b){
    return minus(a,b);
}
template <class num>
std::vector<std::vector<num> > operator-(std::vector<std::vector<num> >A, std::vector<std::vector<num> > B){
    return minus(A,B);
}

template <class num>
std::vector<std::vector<num> > operator*(double alpha, std::vector<std::vector<num> > A){
    return scal(A,alpha);
}
template <class num>
std::vector<num> operator*(double alpha, std::vector<num> a){
    return scal(a,alpha);
}
template <class num>
std::vector<std::vector<num> > operator*(std::vector<std::vector<num> > A, double alpha){
    return scal(A,alpha);
}
template <class num>
std::vector<num> operator*(std::vector<num> a, double alpha){
    return scal(a,alpha);
}

template <class mat>
std::vector<std::vector<mat> > operator*(std::vector<std::vector<mat> > A, std::vector<std::vector<mat> > B){
    return matmul(A,B);
}
template <class mat>
std::vector<mat> operator*(std::vector<std::vector<mat> > A, std::vector<mat> b){
    return matmul(A,b);
}
template <class mat>
std::vector<mat> operator*(std::vector<mat> b, std::vector<std::vector<mat> > A){
    return matmul(b,A);
}


template <class num>
num sum(std::vector<num> x){
    num sumx= 0;
    for (unsigned int i=0;i<x.size();i++){
        sumx+= x.at(i);
    }
    return sumx;
}

template <class num>
num mean(std::vector<num> x){
    num sumx= 0;
    for (unsigned int i=0;i<x.size();i++){
        sumx+= x.at(i);
    }
    return 1.*sumx/x.size();
}

template <class num>
std::vector<num> mean(std::vector<std::vector<num> > A, int axis){
//    axis=0: mean of row | axis=1: mean of column
    std::vector<num> avgVect;
    if (axis==1)
        A= transpose(A);

    for (unsigned int i=0;i<A.size();i++){
        avgVect.push_back( mean(A.at(i)) );
    }
    return avgVect;
}

template <class num>
num min(num a,num b){
    return ( (a<b)?a:b );
}

template <class num>
int nonZero1st(std::vector<num> x){
    for (unsigned int i=0;i<x.size();i++){
        if (x.at(i)>1e-10) return int(i);
    }
    return -1;
}

template <class num>
std::vector<num> reverseVect(std::vector<num> x){
    std::vector<num> y;
    unsigned int xn= x.size();
    for (unsigned int i=0;i<xn;i++){
        y.push_back(x.pop_back());
    }
    return y;
}

template <class num>
std::vector<std::vector<num> > eye(int m, int n, num a=1){
    std::vector<std::vector<num> > A(m, std::vector<num>(n,0));
    for(unsigned int i=0;i<fmin(m,n);i++){
        A.at(i).at(i)=1*a;
    }
    return A;
}

template <class num>
std::vector<num> ei(int m, int i, num a=1){
    std::vector<num> x(m,0);
    x.at(i)=a;
    return x;
}

template <class num>
std::vector<std::vector<num> > diag(std::vector<num> x){
    int m= x.size();
    std::vector<std::vector<num> > A(m,std::vector<num>(m,0));
    for (int i=0;i<m;i++){
        A.at(i).at(i)=x.at(i);
    }
    return A;
}

template <class num>
std::vector<num> diag(std::vector<std::vector<num> > A){
    int m= A.size();
    int n= A.at(0).size();
    std::vector<num> x;
    for (int i=0;i<fmin(m,n);i++){
        x.push_back(A.at(i).at(i) );
    }
    return x;
}

template <class num>
std::vector<num> cross(std::vector<num> a, std::vector<num> b){
    if (a.size()!=3 || b.size()!=3){
        printf("a: %ldx1 crossing b:%ldx1, "
               "taking only the first 3 element\n",a.size(),b.size());
    }
    std::vector<num> c(3,0);
    c.at(0)= -a.at(2)*b.at(1) + a.at(1)*b.at(2);
    c.at(1)= a.at(2)*b.at(0) - a.at(0)*b.at(2);
    c.at(2)= -a.at(1)*b.at(0) + a.at(0)*b.at(1);
    return c;
}

template <class num>
std::vector<std::vector<num> > crossMat(std::vector<num> a){
    std::vector<std::vector<num> > Sa= makeMat<num>(3,3,0);
    Sa.at(0).at(1)= -a.at(2);
    Sa.at(0).at(2)= a.at(1);

    Sa.at(1).at(0)= a.at(2);
    Sa.at(1).at(2)= -a.at(0);

    Sa.at(2).at(0)= -a.at(1);
    Sa.at(2).at(1)= a.at(0);
    return Sa;
}

template <class num>
num dot(std::vector<num> a, std::vector<num> b){
    if (a.size()!=b.size()){
        printf("a:%ldx1 and b:%ldx1 cannot be dotted",a.size(),b.size());
        exit(1);
    }
    num c=0;
    for (unsigned int i=0;i<a.size();i++){
        c+= a.at(i)*b.at(i);
    }
    return c;
}

template <class num>
double norm(std::vector<num> a){
    return dot(a,a);
}

template <class num, class wnum>
double norm(std::vector<num> a, std::vector<wnum> W){
    if (W.size()!=a.size()){
        printf("weighting vector: %ldx1 not compatible with x: %ldx1\n",
               W.size(),a.size());
        exit(1);
    }
    num c=0;
    for (unsigned int i=0;i<a.size();i++){
        c+= a.at(i)*a.at(i)*W.at(i);
    }
    return c;
}

template <class num>
double l2norm(std::vector<num> a){
    return sqrt(dot(a,a));
}

template <class num>
double l2norm(std::vector<num> a, std::vector<num> W){
    return sqrt(norm(a,W));
}

template <class num>
std::vector<num> normalize(std::vector<num> a){
    return (1./l2norm(a))*a;
}

template <class num>
std::vector<num> project(std::vector<num> a, std::vector<num> b){
    return scal(b, dot(a,b)/norm(b));
}

template <class num>
std::vector<double> rand_d(int m){
    int max_num= RAND_MAX;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(-max_num,max_num);

    std::vector<double> x;
    for(int i=0; i<m;i++){
//        x.push_back(double(rand())/double(RAND_MAX)-0.5);
        x.push_back(double(dis(gen))/double(max_num));
    }
    return x;
}

template <class num>
double det_2x2ana(std::vector<std::vector<num> >A){
    unsigned int am,an;
    am= A.size(); an= A.at(0).size();
    if (am!=2 || an!=2){
        printf("A: %dx%d cannot use det_2x2ana\n",am,an);
        exit(1);
    }
    double detA= A.at(0).at(0)*A.at(1).at(1)-A.at(0).at(1)*A.at(1).at(0);
    return detA;
}

template <class num>
double det_3x3ana(std::vector<std::vector<num> >A){
    unsigned int am,an;
    am= A.size(); an= A.at(0).size();
    if (am!=3 || an!=3){
        printf("A: %dx%d cannot use det_3x3ana\n",am,an);
        exit(1);
    }

    num a11,a12,a13;
    double detA;
    a11= A[1][1]*A[2][2] - A[2][1]*A[1][2];
    a12= A[1][0]*A[2][2] - A[2][0]*A[1][2];
    a13= A[1][0]*A[2][1] - A[2][0]*A[1][1];

//    num a21,a22,a23, a31,a32,a33;
//    a21= A[0][1]*A[2][2] - A[2][1]*A[0][2];
//    a22= A[0][0]*A[2][2] - A[2][0]*A[0][2];
//    a23= A[0][0]*A[2][1] - A[2][0]*A[0][1];

//    a31= A[0][1]*A[1][2] - A[0][2]*A[1][1];
//    a32= A[0][0]*A[1][2] - A[0][2]*A[1][0];
//    a33= A[0][0]*A[1][1] - A[0][1]*A[1][0];

    detA= A.at(0).at(0)*a11 - A.at(0).at(1)*a12 + A.at(0).at(2)*a13;
    return detA;
}

template <class num>
std::vector<std::vector<num> > inv_3x3ana(std::vector<std::vector<num> > A){
    unsigned int am,an;
    am= A.size(); an= A.at(0).size();
    if (am!=3 || an!=3){
        printf("A: %dx%d cannot use inv_3x3ana\n",am,an);
        exit(1);
    }
    std::vector<std::vector<num> > Ainv(3, std::vector<num>(3,0));
    num a11,a12,a13, a21,a22,a23, a31,a32,a33;
    double detA;
    a11= A[1][1]*A[2][2] - A[2][1]*A[1][2];
    a12= A[1][0]*A[2][2] - A[2][0]*A[1][2];
    a13= A[1][0]*A[2][1] - A[2][0]*A[1][1];

    a21= A[0][1]*A[2][2] - A[2][1]*A[0][2];
    a22= A[0][0]*A[2][2] - A[2][0]*A[0][2];
    a23= A[0][0]*A[2][1] - A[2][0]*A[0][1];

    a31= A[0][1]*A[1][2] - A[0][2]*A[1][1];
    a32= A[0][0]*A[1][2] - A[0][2]*A[1][0];
    a33= A[0][0]*A[1][1] - A[0][1]*A[1][0];

    detA= A.at(0).at(0)*a11 - A.at(0).at(1)*a12 + A.at(0).at(2)*a13;

    std::vector<num> buf0(3,0), buf1(3,0), buf2(3,0);
    buf0.at(0)= a11/detA;   buf0.at(1)= -a21/detA;   buf0.at(2)= a31/detA;
    buf1.at(0)= -a12/detA;   buf1.at(1)= a22/detA;   buf1.at(2)= -a32/detA;
    buf2.at(0)= a13/detA;   buf2.at(1)= -a23/detA;   buf2.at(2)= a33/detA;

    Ainv.at(0)= buf0;
    Ainv.at(1)= buf1;
    Ainv.at(2)= buf2;
    return Ainv;
}

template <class num>
void bidiag(std::vector<std::vector<num> >A, std::vector<std::vector<num> >&U, std::vector<std::vector<num> >&T, std::vector<std::vector<num> >&V){
    // Golub-Kahan
    int m= A.size();
    int n= A.at(0).size();

    if (m<n){
        bidiag(transpose(A), U,T,V);
        std::vector<std::vector<num> > temp= transpose(V);
        V= transpose(U);
        U= temp;
        T= transpose(T);
        return;
    }

    U= eye<num>(m,m);
    V= eye<num>(n,n);

    std::vector<std::vector<num> > Im, In;
    Im= eye<num>(m,m);
    In= eye<num>(n,n);

    for (unsigned int i=0;i<m;i++){
        for (int j=0;j<m-n;j++)
            A.at(i).push_back(0);
    }

    for (int i=0;i<n;i++){
        std::vector<num> um(m,0);
        double alpha= -sign(A.at(i).at(i))* l2norm(extractVect(A,i,-1,i,i));
        double r= sqrt(2*(alpha*alpha - A.at(i).at(i)*alpha));

        std::vector<std::vector<num> > uuT, Pi;
        std::vector<num> uTA;
        if (fabs(r)>1e-10){

            um.at(i)= ( A.at(i).at(i)-alpha )/r;
            for (int j=i+1;j<m;j++){
                um.at(j)= A.at(j).at(i)/r;
            }

            uTA= matmul(um,A);
            uuT= outermul(um,um);

            A= add(A, outermul(um,uTA), 1,-2);

            Pi= minus(Im, uuT, 1,-2);
            U= matmul(Pi,U);
        }

        if (i<n-2){
            std::vector<num> un(m,0);
            double alpha= -sign(A.at(i).at(i+1))* l2norm(extractVect(A,i,i,i+1,-1));
            double r= sqrt(2*(alpha*alpha - A.at(i).at(i+1)*alpha));
            if (fabs(r)<1e-10) continue;

            un.at(i+1)= ( A.at(i).at(i+1)-alpha )/r;
            for (int j=i+2;j<n;j++){
                un.at(j)= A.at(i).at(j)/r;
            }

            std::vector<num> Au= matmul(A,un);
            uuT= outermul(un,un);

            A= add(A, outermul(Au,un), 1,-2);

            Pi= minus(In, uuT, 1,-2);
            V= matmul(V,Pi);

        }
    }
    T= A;
}

template <class num>
std::vector<std::vector<num> > hhRed(std::vector<std::vector<num> > A, std::vector<std::vector<num> > &P, std::vector<std::vector<num> > &D){
    int m= A.size();
    int n= A.at(0).size();
    std::vector<std::vector<num> > Im= eye<num>(m,m);
    P= eye<num>(m,n);

    for (int i=0;i<n-2;i++){
        std::vector<num> u(m,0);
        double alpha= -sign(A.at(i+1).at(i))* l2norm(extractVect(A,i+1,-1,i,i));
        double r= sqrt(2*(alpha*alpha - A.at(i+1).at(i)*alpha));
        if (fabs(r)<1e-10) continue;

        u.at(i+1)= ( A.at(i+1).at(i)-alpha )/r;
        for (int j=i+2;j<m;j++){
            u.at(j)= A.at(j).at(i)/r;
        }

        std::vector<std::vector<num> > uuT, Pi;
        std::vector<num> uTA, Au;
        num uTAu;
        uTA= matmul(u,A);
        Au= matmul(A,u);
        uTAu= dot(u,Au);
        uuT= outermul(u,u);

        A= add(A, outermul(u,uTA), 1,-2);
        A= add(A, outermul(Au,u), 1,-2);
        A= add(A, uuT, 1, 4*uTAu);

        Pi= minus(Im, uuT, 1,-2);
        P= matmul(Pi,P);
    }

    D= A;
    return D;
}

template <class num>
std::vector<std::vector<num> > gramSchmidt(std::vector<std::vector<num> > A, std::vector<std::vector<num> > &Q, std::vector<std::vector<num> > &R){
    Q= std::vector<std::vector<num> >();
    R= std::vector<std::vector<num> >();

    unsigned int an= A.at(0).size();
    for (unsigned int i=0,k=0;i<an;i++,k++){
        Q.push_back(extractVect(A,0,-1,i,i));
        std::vector<num> ri;
        for (int j=0;j<signed(k);j++){
            double project= double (dot(Q.at(k),Q.at(j)) );
            Q.at(k)= add(Q.at(k), Q.at(j), 1, -project/dot(Q.at(j),Q.at(j)));
            ri.push_back(project);
        }

        double unitFactor= sqrt(dot(Q.at(k),Q.at(k)) );
        if (fabs(unitFactor) < 1e-10){
//            printf("Matrix is NOT linearly independent!\n");
            Q.pop_back();
            k-=1;
        }else{
            ri.push_back(unitFactor);
            Q.at(k)= scal(Q.at(k), 1/unitFactor);
        }
        R.push_back(ri);
    }

    Q= transpose(Q);
    R= transpose(R);
    return Q;
}

template <class num>
std::vector<std::vector<num> > gramSchmidtPivot(std::vector<std::vector<num> >A,
                                                std::vector<std::vector<num> >&Q, std::vector<std::vector<num> >&R,
                                                std::vector<std::vector<num> >&P){
    std::vector<std::vector<num> > AT,AP;
    std::vector<num> a_norm, ind;
    AT= transpose(A);
    for (unsigned int i=0;i<AT.size();i++){
        unsigned int k=0;
        num mag= norm(AT.at(i));
        for (unsigned int j=0;j<i;j++){
            if (mag>a_norm.at(j)){
                break;
            }
            k++;
        }
        a_norm.insert(a_norm.begin()+k,mag);
        ind.insert(ind.begin()+k,i);
    }

    P= makeMat<num>(AT.size(),AT.size(),0);
    for (unsigned int i=0;i<AT.size();i++){
        P.at(i).at(ind.at(i))=1;
    }
    AP= matmul(A,P);
    return gramSchmidt(AP,Q,R);
}

template <class num>
std::vector<std::vector<num> > padOrtho(std::vector<std::vector<num> > Q, int numNull){
    int qm= int(Q.size());
    int qn= int(Q.at(0).size());
    Q= transpose(Q);

    std::vector<double> ni;
    std::vector<std::vector<num> > nullSpace;

    int p=0;
    for(int j=0;numNull>0;){
        if (p<qm){
            ni= ei<double>(qm,p);
            p++;
        }else{
            ni= rand_d<double>(qm);
        }

        for (int i=0;i<qn;i++){
            ni= add(ni, Q.at(i), 1, double(-dot(ni,Q.at(i)) )/dot(Q.at(i),Q.at(i)) );
        }
        for (int k=0;k<j;k++){
            ni= add(ni, nullSpace.at(k), 1, double(-dot(ni,nullSpace.at(k)) )/dot(nullSpace.at(k),nullSpace.at(k)) );
        }
        double unitFactor= sqrt(dot(ni,ni) );
        if (fabs(unitFactor) >1e-10){
            nullSpace.push_back(scal(ni,1/unitFactor));
            numNull-=1;
            j++;
        }
    }
    nullSpace= transpose(nullSpace);
    return nullSpace;
}

template <class num>
std::vector<std::vector<num> > eigen(std::vector<std::vector<num> > A, std::vector<std::vector<num> > &Q, std::vector<std::vector<num> > &Lamda){
    std::vector<std::vector<num> > P, D, Qi,Ri;
    hhRed(A,P,D);

    gramSchmidt(D,Qi,Ri);
    Q= Qi;

    int i=0;
    unsigned int zero_num=0;
    while(zero_num<min(Ri.size(),Ri.at(0).size()) && i<70000){
        gramSchmidt(matmul(Ri,Qi), Qi,Ri);
        Q= matmul(Q,Qi);

        zero_num= 0;
        std::vector<num> Ri_j;
        for(unsigned int j=0;j<min(Ri.size(),Ri.at(0).size());j++){
            Ri_j= Ri.at(j);
            Ri_j.at(j)=0;
            if (norm(Ri_j)>1e-10){
                break;
            }
            zero_num++;
        }

        i++;
    }
    Lamda= matmul(Ri,Qi);
    Q= matmul( transpose(P),Q );

    return Lamda;
}

template <class num>
int svd(std::vector<std::vector<num> >A, std::vector<std::vector<num> >&U, std::vector<std::vector<num> >&S,std::vector<std::vector<num> >&V){
    int m= A.size();
    int n= A.at(0).size();

    std::vector<std::vector<num> > u,T,v, Lambda, invS;
    bidiag(A, u,T,v); //uAv=T -> A= u'*T*v'
    eigen(matmul(transpose(T),T), V,Lambda);

    S= makeMat<num>(m,n,0);
    invS= makeMat<num>(n,m,0);
    for (unsigned int i=0; i<Lambda.size();i++){
        double si= sqrt(Lambda.at(i).at(i));
        S.at(i).at(i)= si;
        invS.at(i).at(i)= 1/si;
    }

    int padN= invS.size()-V.at(0).size();
    if (padN>0){
        for (unsigned int i=0;i<V.size();i++){
            for (int j=0;j<padN;j++)
                V.at(i).push_back(0);
        }
    }

    int padU= T.size()-u.size();
    for (int i=0;i<padU;i++){
        u.push_back( std::vector<double>(u.at(0).size(),0) );
    }

    int padv= V.size()-v.at(0).size();
    for (unsigned int i=0;i<v.size()&&padv>0;i++){
        for (int j=0;j<padv;j++)
            v.at(i).push_back(0);
    }

    U= transpose(u) * T*(V*invS);
    V= matmul(v, V);

//    A=U*S*V'
    return 1;
}

template <class num>
int null(std::vector<std::vector<num> > A, std::vector<std::vector<num> > &nullSpace, double thres=0){
//    A*nullSpace=0
    std::vector<std::vector<num> > Q,R,P, U,S,V;
    nullSpace= std::vector<std::vector<num> >();

    svd(A,U,S,V);

    int numNull= int(A.at(0).size()-V.at(0).size());
    if (numNull==0 && thres<=0){
        return -1;
    }
    if (numNull>0){
        nullSpace= padOrtho(V,numNull);
    }

    if (thres>0){
        nullSpace= transpose(nullSpace);
        int m= V.size();
        int n= V.at(0).size();
        V= transpose(V);
        for (int i=min(m,n-1);i>=0;i--){
            if (fabs(S.at(i).at(i))<thres){
                nullSpace.push_back(V.at(i));
            }
        }
        nullSpace= transpose(nullSpace);
    }

//    print_matrix("Anull",A*nullSpace);
//    print_matrix("null",nullSpace);
    return 1;
}

template <class num>
int null_mxm(std::vector<std::vector<num> > A, std::vector<std::vector<num> > &nullSpace, double thres=0){
    std::vector<std::vector<num> > null_ortho;
    if (null(A,null_ortho,thres)==-1)
        return -1;

    nullSpace= std::vector<std::vector<num> > ();
    unsigned int m= null_ortho.size();
    unsigned int n= null_ortho.at(0).size();
    null_ortho= transpose(null_ortho);
    for (unsigned int i=0;i<m;i++){
        std::vector<num> null_i(m,0);
        for (unsigned int j=0;j<n;j++){
            null_i= null_i + null_ortho.at(j).at(i)*null_ortho.at(j);
        }
        nullSpace.push_back(null_i);
    }

    return 1;
}

template <class num>
std::vector<std::vector<num> > pinv(std::vector<std::vector<num> > A, double thres=0){
    unsigned int m,n,mn_min;
    m= A.size();
    n= A.at(0).size();
    mn_min= min(m,n);
    std::vector<std::vector<num> > Apinv, U,S,V, Sinv;
    svd(A,U,S,V);

    Sinv= makeMat<num>(n,m,0);
    for (unsigned int i=0;i<mn_min;i++){
        if (fabs(S.at(i).at(i))>thres){
            Sinv.at(i).at(i)= 1.0/S.at(i).at(i);
        }
    }
    Apinv= V*Sinv*transpose(U);
    return Apinv;
}

template <class num>
double detSym(std::vector<std::vector<num> > A){
    double detA= 1;
    std::vector<std::vector<num> > Q,R,P;
    gramSchmidtPivot(A,Q,R,P);
    for (unsigned int i=0;i<R.size();i++){
        detA*= R.at(i).at(i);
    }
    return detA;
}

template <class num>
std::vector<num> lsqQR(std::vector<std::vector<num> > A, std::vector<num> y, double thres=0){
    std::vector<std::vector<num> > Q,R;

    gramSchmidt(A,Q,R);
    int rm= int(R.size());
    int rn= int(R.at(0).size());
    std::vector<num> x(rn,0), Qy , ri, xi;

    if (thres>0){
        std::vector<std::vector<num> > nullSpace;
        null(transpose(A),nullSpace,thres);
        nullSpace= transpose(nullSpace);
        for(unsigned int i=0;i<nullSpace.size();i++){
            y= y-project(y,nullSpace.at(i));
            print_vector("y",y);
        }
    }
    Qy= matmul(transpose(Q),y);

    int k= rm-1;
    if (rm==rn){
        x.at(rm-1)= Qy.at(rm-1)/R.at(rm-1).at(rm-1);
        k-=1;
    }

    for(int i=k;i>=0;i--){
        ri= extractVect(R,i,i,i,-1);
//        ri= R.at(i);
        xi= extractVect(x,i,-1);
        int j= nonZero1st(ri);
        if (j==-1){
            printf("Whole Row of R%d is zero\n",i);
        }
        x.at(i+j)= Qy.at(i)- dot(extractVect(R,i,i,i,-1),extractVect(x,i,-1));
        x.at(i+j)= x.at(i+j)/ri.at(j);
    }

    return x;
}

template <class num>
std::vector<std::vector<num> > pinv_lsq(std::vector<std::vector<num> > A){
    unsigned int m= A.size();
    std::vector<std::vector<num> > pinv;

    for (unsigned int i=0;i<m;i++){
        std::vector<num> ei(m,0);
        ei.at(i)=1;
        std::vector<num> pinv_i= lsqQR(A,ei);
        pinv.push_back(pinv_i);
    }

    return transpose(pinv);
}


template <class num>
std::vector<num> dls(std::vector<std::vector<num> > A, std::vector<num> y, num lambda){
    std::vector<std::vector<num> > AAT, lamI, B;
    std::vector<num> f, x;
    int m= A.size();
    AAT= matmul( A, transpose(A) );
    lamI= eye<num>(m,m, lambda*lambda);
    B= add(AAT, lamI);

    f= lsqQR(B,y);
    x= matmul( transpose(A),f);
    return x;
}


template <class num>
std::vector<std::vector<num> > J_numerical(std::vector<num> (*f)(std::vector<num>), std::vector<num> x){
    std::vector<std::vector<num> > J;
    double delta= 0.0000001;
    for (unsigned int i=0;i<x.size();i++){
        std::vector<num> xi_p,xi_m, Ji;
        xi_p=x;
        xi_m=x;
        xi_p.at(i)+=delta;
        xi_m.at(i)-=delta;
        Ji= (f(xi_p)-f(xi_m))*(0.5/delta) ;
        J.push_back(Ji);
    }
    J= transpose(J);
    return J;
}

template <class num>
std::vector<num> gradDescent(std::vector<num>(*f)(std::vector<num>), std::vector<num> y, std::vector<num> x_init,
                             std::vector<std::vector<num> > (*J)(std::vector<num>), double gamma=0.01, double thres=1e-8){

    std::vector<num> x= x_init;
    for (int i=0;i<1000000;i++){
        x= x - gamma*(transpose(J(x))*(f(x)-y));
        if (l2norm(f(x)-y)<thres){
            return x;
        }
    }
    return x;
}

template <class num>
std::vector<num> gradDescent(std::vector<num>(*f)(std::vector<num>), std::vector<num> x_init,
                             std::vector<std::vector<num> > (*J)(std::vector<num>), double gamma=0.01, double thres=1e-8){

    std::vector<num> x= x_init;
    for (int i=0;i<1000000;i++){
        x= x + gamma*(transpose(J(x))*f(x));
        if (l2norm(f(x))<thres){
            return x;
        }
    }
    return x;
}


template <class num>
std::vector<num> gradDescent(boost::function<std::vector<num> (std::vector<num>)> f, std::vector<num> y, std::vector<num> x_init,
                             boost::function<std::vector<std::vector<num> > (std::vector<num>)> J, double gamma=0.01, double thres=1e-8){
    std::vector<num> x= x_init;
    for (int i=0;i<1000000;i++){
        x= x - gamma*(transpose(J(x))*(f(x)-y));
        if (l2norm(f(x)-y)<thres){
            return x;
        }
    }
    return x;
}

template <class num>
std::vector<num> gradDescent(boost::function<std::vector<num> (std::vector<num>)> f, std::vector<num> x_init,
                             boost::function<std::vector<std::vector<num> > (std::vector<num>)> J, double gamma=0.01, double thres=1e-8){
    std::vector<num> x= x_init;
    for (int i=0;i<1000000;i++){
        x= x + gamma*(transpose(J(x))*f(x));
//        std::vector<std::vector<num> > Jx= extractMat(J(x),3,5,0,-1);
//        x= x + gamma*(transpose(Jx)*f(x));

        if (l2norm(f(x))<thres){
            return x;
        }

        if (i%10000==0){
            printf("i: %d  |norm: %3.3f\n",i,l2norm(f(x)));
            print_vector("x",x);
            print_vector("f(x)",f(x));
            printf("\n");
        }
    }
    return x;
}


#endif // UTIL_H
