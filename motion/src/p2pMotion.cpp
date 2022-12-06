
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "kinematics.h"

void p2pMotionPlan(frame start, frame goal, float minT, float maxT, float dt)
{
    Eigen::MatrixXf Q0 = invKin(start);
    Eigen::MatrixXf Qf = invKin(goal);

    Eigen::VectorXf q0(6);
    q0 << Q0.block(0,0,1,6);
    Eigen::VectorXf qf(6);
    qf << Qf.block(0,0,1,6);
    Eigen::Matrix4f M;
    Eigen::Vector4f b;
    Eigen::Vector4f a;
    Eigen::MatrixXf A(1,4);

    for(int i = 0; i < 6; i++)
    {
        M << 1, minT, pow(minT, 2), pow(minT, 3),
            0, 1, 2*minT, 3*pow(minT, 2),
            1, maxT, pow(maxT, 2), pow(maxT, 3),
            0, 1, 2*maxT, 3*pow(maxT, 2);
        b << q0(i), 0, qf(i), 0;
        a = M.inverse()*b;
        A.row(A.rows() - 1) << a.transpose();
        A.conservativeResize(A.rows() + 1, A.cols());
    }

    Eigen::MatrixXf TH(1,6);
    Eigen::MatrixXf xE(1,4);
    Eigen::MatrixXf phiE(1,4);

    

}

int main(int argc, char **argv)
{

}