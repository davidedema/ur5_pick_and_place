#ifndef DIFFKIN_H
#define DIFFKIN_H

#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

typedef Matrix <float, 1, 6> Array6d;

void ur5Jac(Matrix <float, 6, 6> &J, Array6d th){
    //same structure as matlab 
    Array6d A;
    A << 0, -0.425, -0.3922, 0, 0, 0;
    Array6d D;
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;

    //first column of the jacobian matrix

    J(0,0) = D(4)*(cos(th(0))*cos(th(4)) + cos(th(1) + th(2) + th(3))*sin(th(0))*sin(th(4))) + D(2)*cos(th(0)) + D(3)*cos(th(0)) - A(2)*cos(th(1) + th(2))*sin(th(0)) - A(1)*cos(th(1))*sin(th(0)) - D(4)*sin(th(1) + th(2) + th(3))*sin(th(0));

    J(1,0) = D(4)*(cos(th(4))*sin(th(0)) - cos(th(1) + th(2) + th(3))*cos(th(0))*sin(th(4))) + D(2)*sin(th(0)) + D(3)*sin(th(0)) + A(2)*cos(th(1) + th(2))*cos(th(0)) + A(1)*cos(th(0))*cos(th(1)) + D(4)*sin(th(1) + th(2) + th(3))*cos(th(0));
     
    J(2,0) = 0;

    J(3,0) = 0;

    J(4,0) = 0;

    J(5,0) = 1;

    //second column

    J(0,1) = -cos(th(0))*(A(2)*sin(th(1) + th(2)) + A(1)*sin(th(1)) + D(4)*(sin(th(1) + th(2))*sin(th(3)) - cos(th(1) + th(2))*cos(th(3))) - D(4)*sin(th(4))*(cos(th(1) + th(2))*sin(th(3)) + sin(th(1) + th(2))*cos(th(3))));
    
    J(1,1) = -sin(th(1))*(A(2)*sin(th(1) + th(2)) + A(1)*sin(th(1)) + D(4)*(sin(th(1) + th(2))*sin(th(3)) - cos(th(1) + th(2))*cos(th(3))) - D(4)*sin(th(4))*(cos(th(1) + th(2))*sin(th(3)) + sin(th(1) + th(2))*cos(th(3))));
    
    J(2,1) = A(2)*cos(th(1) + th(2)) - (D(4)*sin(th(1) + th(2) + th(3) + th(4)))/2 + A(1)*cos(th(1)) + (D(4)*sin(th(1) + th(2) + th(3) - th(4)))/2 + D(4)*sin(th(1) + th(2) + th(3));
    
    J(3,1) = sin(th(0));
    
    J(4,1) = -cos(th(0));
    
    J(5,1) = 0;


    //third 

    J(0,2) = cos(th(0))*(D(4)*cos(th(1) + th(2) + th(3)) - A(2)*sin(th(1) + th(2)) + D(4)*sin(th(1) + th(2) + th(3))*sin(th(4)));

    J(1,2) = sin(th(0))*(D(4)*cos(th(1) + th(2) + th(3)) - A(2)*sin(th(1) + th(2)) + D(4)*sin(th(1) + th(2) + th(3))*sin(th(4)));
     
    J(2,2) = A(2)*cos(th(1) + th(2)) - (D(4)*sin(th(1) + th(2) + th(3) + th(4)))/2 + (D(4)*sin(th(1) + th(2) + th(3) - th(4)))/2 + D(4)*sin(th(1) + th(2) + th(3));
     
    J(3,2) = sin(th(0));

    J(4,2) = -cos(th(0));

    J(5,2) = 0;

    //forth

    J(0,3) = D(4)*cos(th(0))*(cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3))*sin(th(4)));
     
    J(1,3) = D(4)*sin(th(0))*(cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3))*sin(th(4)));
    
    J(2,3) = D(4)*(sin(th(1) + th(2) + th(3) - th(4))/2 + sin(th(1) + th(2) + th(3)) - sin(th(1) + th(2) + th(3) + th(4))/2);

    J(3,3) = sin(th(0));

    J(4,3) = -cos(th(0));

    J(5,3) = 0;

    //fifth

    J(0,4) = -D(4)*sin(th(0))*sin(th(4)) - D(4)*cos(th(1) + th(2) + th(3))*cos(th(0))*cos(th(4));
    
    J(1,4) = D(4)*cos(th(0))*sin(th(4)) - D(4)*cos(th(1) + th(2) + th(3))*cos(th(4))*sin(th(0));

    J(2,4) = -D(4)*(sin(th(1) + th(2) + th(3) - th(4))/2 + sin(th(1) + th(2) + th(3) + th(4))/2);
    
    J(3,4) = sin(th(1) + th(2) + th(3))*cos(th(0));

    J(4,4) = sin(th(1) + th(2) + th(3))*sin(th(2));

    J(5,4) = -cos(th(1) + th(2) + th(3));

    //sixth

    J(0,5) = 0;

    J(1,5) = 0;

    J(2,5) = 0;

    J(3,5) = cos(th(4))*sin(th(0)) - cos(th(1) + th(2) + th(3))*cos(th(0))*sin(th(4));

    J(4,5) = -cos(th(0))*cos(th(4)) - cos(th(1) + th(2) + th(3))*sin(th(0))*sin(th(4));
    
    J(5,5) = -sin(th(1) + th(2) + th(3))*sin(th(4));

}


#endif
