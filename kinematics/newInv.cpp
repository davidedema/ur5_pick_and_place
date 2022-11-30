#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h> 
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <complex>

//-----------------------------------------------//
struct dkin
{
    Eigen::Matrix3f rot;
    Eigen::Vector3f xyz;
};

Eigen::Matrix4f t10f(std::complex<float> th1)
{
    Eigen::Matrix4f aux;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << std::real(cos(th1)), std::real(-sin(th1)), 0, 0,
        std::real(sin(th1)), std::real(cos(th1)), 0, 0,
        0, 0, 1, d(0),
        0, 0, 0, 1;

    return aux;
}

Eigen::Matrix4f t21f(std::complex<float> th2)
{
    Eigen::Matrix4f aux;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << std::real(cos(th2)), std::real(-sin(th2)), 0, 0,
        0, 0, -1, 0,
        std::real(sin(th2)), std::real(cos(th2)), 0, 0,
        0, 0, 0, 1;

    return aux;
}

Eigen::Matrix4f t32f(std::complex<float> th3)
{
    Eigen::Matrix4f aux;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << std::real(cos(th3)), std::real(-sin(th3)), 0, a(1),
        std::real(sin(th3)), std::real(cos(th3)), 0, 0,
        0, 0, 1, d(2),
        0, 0, 0, 1;

    return aux;
}

Eigen::Matrix4f t43f(std::complex<float> th4)
{
    Eigen::Matrix4f aux;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << std::real(cos(th4)), std::real(-sin(th4)), 0, a(2),
        std::real(sin(th4)), std::real(cos(th4)), 0, 0,
        0, 0, 1, d(3),
        0, 0, 0, 1;

    return aux;
}

Eigen::Matrix4f t54f(std::complex<float> th5)
{
    Eigen::Matrix4f aux;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << std::real(cos(th5)), std::real(-sin(th5)), 0, 0,
        0, 0, -1, -d(4),
        std::real(sin(th5)), std::real(cos(th5)), 0, 0,
        0, 0, 0, 1;

    return aux;
}

Eigen::Matrix4f t65f(std::complex<float> th6)
{
    Eigen::Matrix4f aux;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << std::real(cos(th6)), std::real(-sin(th6)), 0, 0,
        0, 0, 1, d(5),
        std::real(-sin(th6)), std::real(-cos(th6)), 0, 0,
        0, 0, 0, 1;

    return aux;
}

/**
 * @brief compute the direct kinematics
 *
 * @param th joint angles
 * @return dkin : rot matrix and cartesian point (respect of the base frame i think)
 */
dkin directKin(Eigen::VectorXf th)
{
    dkin ret;
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;

    Eigen::Matrix4f t60 = t10f(th(0)) * t21f(th(1)) * t32f(th(2)) * t43f(th(3)) * t54f(th(4)) * t65f(th(5));

    ret.rot = t60.block(0, 0, 3, 3);
    ret.xyz = t60.block(0, 3, 3, 1);

    return ret;
}

Eigen::MatrixXf invKin(dkin &frame)
{
    // Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    // Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;

    Eigen::Matrix4f t60;

    // Matrix creation

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            t60(i, j) = frame.rot(i, j);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        t60(i, 3) = frame.xyz(i);
    }

    for (int j = 0; j < 3; j++)
    {
        t60(3, j) = 0;
    }

    t60(3, 3) = 1;

    // finding th1

    Eigen::Vector4f aux;
    aux << 0, 0, -d(5), 1;
    Eigen::Vector4f p50;
    p50 = t60 * aux;
    std::complex<float> th1_1 = std::real(atan2(p50(1), p50(0)) + acos(d(3) / hypot(p50(1), p50(0)))) + M_PI / 2;
    std::complex<float> th1_2 = std::real(atan2(p50(1), p50(0)) - acos(d(3) / hypot(p50(1), p50(0)))) + M_PI / 2;

    // finding th5

    std::complex<float> th5_1 = +std::real(acos((frame.xyz(0) * std::real(sin(th1_1)) - frame.xyz(1) * std::real(cos(th1_1)) - d(3)) / d(5)));
    std::complex<float> th5_2 = -std::real(acos((frame.xyz(0) * std::real(sin(th1_1)) - frame.xyz(1) * std::real(cos(th1_1)) - d(3)) / d(5)));
    std::complex<float> th5_3 = +std::real(acos((frame.xyz(0) * std::real(sin(th1_2)) - frame.xyz(1) * std::real(cos(th1_2)) - d(3)) / d(5)));
    std::complex<float> th5_4 = -std::real(acos((frame.xyz(0) * std::real(sin(th1_2)) - frame.xyz(1) * std::real(cos(th1_2)) - d(3)) / d(5)));

    // related to th11 a th51
    Eigen::Matrix4f t06;
    t06 = t60.inverse();
    Eigen::Vector3f Xhat;
    Xhat = t06.block(0, 0, 3, 1);
    Eigen::Vector3f Yhat;
    Yhat = t06.block(0, 1, 3, 1);

    std::complex<float> th6_1 = std::real(atan2(((-Xhat(1) * std::real(sin(th1_1)) + Yhat(1) * std::real(cos(th1_1)))) / std::real(sin(th5_1)), ((Xhat(0) * std::real(sin(th1_1)) - Yhat(0) * std::real(cos(th1_1)))) / std::real(sin(th5_1))));

    // related to th11 a th52
    std::complex<float> th6_2 = std::real(atan2(((-Xhat(1) * std::real(sin(th1_1)) + Yhat(1) * std::real(cos(th1_1)))) / std::real(sin(th5_2)), ((Xhat(0) * std::real(sin(th1_1)) - Yhat(0) * std::real(cos(th1_1)))) / std::real(sin(th5_2))));

    // related to th12 a th53
    std::complex<float> th6_3 = std::real(atan2(((-Xhat(1) * std::real(sin(th1_2)) + Yhat(1) * std::real(cos(th1_2)))) / std::real(sin(th5_3)), ((Xhat(0) * std::real(sin(th1_2)) - Yhat(0) * std::real(cos(th1_2)))) / std::real(sin(th5_3))));

    // related to th12 a th54
    std::complex<float> th6_4 = std::real(atan2(((-Xhat(1) * std::real(sin(th1_2)) + Yhat(1) * std::real(cos(th1_2)))) / std::real(sin(th5_4)), ((Xhat(0) * std::real(sin(th1_2)) - Yhat(0) * std::real(cos(th1_2)))) / std::real(sin(th5_4))));

    Eigen::Matrix4f t41m;
    Eigen::Vector3f p41_1;
    Eigen::Vector3f p41_2;
    Eigen::Vector3f p41_3;
    Eigen::Vector3f p41_4;
    float p41xz_1;
    float p41xz_2;
    float p41xz_3;
    float p41xz_4;

    t41m = t10f(std::real(th1_1)).inverse() * t60 * t65f(std::real(th6_1)).inverse() * t54f(std::real(th5_1)).inverse();
    p41_1 = t41m.block(0, 3, 3, 1);
    p41xz_1 = hypot(p41_1(0), p41_1(2));

    t41m = t10f(std::real(th1_1)).inverse() * t60 * t65f(std::real(th6_2)).inverse() * t54f(std::real(th5_2)).inverse();
    p41_2 = t41m.block(0, 3, 3, 1);
    p41xz_2 = hypot(p41_2(0), p41_2(2));

    t41m = t10f(std::real(th1_2)).inverse() * t60 * t65f(std::real(th6_3)).inverse() * t54f(std::real(th5_3)).inverse();
    p41_3 = t41m.block(0, 3, 3, 1);
    p41xz_3 = hypot(p41_3(0), p41_3(2));

    t41m = t10f(std::real(th1_2)).inverse() * t60 * t65f(std::real(th6_4)).inverse() * t54f(std::real(th5_4)).inverse();
    p41_4 = t41m.block(0, 3, 3, 1);
    p41xz_4 = hypot(p41_4(0), p41_4(2));

    // computation of eight possible value for th3
    std::complex<float> th3_1 = std::real(acos((pow(p41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2))));
    std::complex<float> th3_2 = std::real(acos((pow(p41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2))));
    std::complex<float> th3_3 = std::real(acos((pow(p41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2))));
    std::complex<float> th3_4 = std::real(acos((pow(p41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2))));

    std::complex<float> th3_5 = -std::real(th3_1);
    std::complex<float> th3_6 = -std::real(th3_2);
    std::complex<float> th3_7 = -std::real(th3_3);
    std::complex<float> th3_8 = -std::real(th3_4);

    // computation of eight possible value for th2
    std::complex<float> th2_1 = std::real(atan2(-p41_1(2), -p41_1(0)) - std::real(asin((-a(2) * std::real(sin(th3_1))) / p41xz_1)));
    std::complex<float> th2_2 = std::real(atan2(-p41_2(2), -p41_2(0)) - std::real(asin((-a(2) * std::real(sin(th3_2))) / p41xz_2)));
    std::complex<float> th2_3 = std::real(atan2(-p41_3(2), -p41_3(0)) - std::real(asin((-a(2) * std::real(sin(th3_3))) / p41xz_3)));
    std::complex<float> th2_4 = std::real(atan2(-p41_4(2), -p41_4(0)) - std::real(asin((-a(2) * std::real(sin(th3_4))) / p41xz_4)));

    std::complex<float> th2_5 = std::real(atan2(-p41_1(2), -p41_1(0)) - std::real(asin((a(2) * std::real(sin(th3_1))) / p41xz_1)));
    std::complex<float> th2_6 = std::real(atan2(-p41_2(2), -p41_2(0)) - std::real(asin((a(2) * std::real(sin(th3_2))) / p41xz_2)));
    std::complex<float> th2_7 = std::real(atan2(-p41_3(2), -p41_3(0)) - std::real(asin((a(2) * std::real(sin(th3_3))) / p41xz_3)));
    std::complex<float> th2_8 = std::real(atan2(-p41_4(2), -p41_4(0)) - std::real(asin((a(2) * std::real(sin(th3_4))) / p41xz_4)));

    Eigen::Matrix4f t43m;
    Eigen::Vector3f xhat43;
    t43m = t32f(th3_1).inverse() * t21f(th2_1).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_1 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_2).inverse() * t21f(th2_2).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_2 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_3).inverse() * t21f(th2_3).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_3 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_4).inverse() * t21f(th2_4).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_4 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_5).inverse() * t21f(th2_5).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_5 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_6).inverse() * t21f(th2_6).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_6 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_7).inverse() * t21f(th2_7).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_7 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_8).inverse() * t21f(th2_8).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    std::complex<float> th4_8 = std::real(atan2(xhat43(1), xhat43(0)));

    Eigen::MatrixXf ret(8, 6);
    ret << std::real(th1_1), std::real(th2_1), std::real(th3_1), std::real(th4_1), std::real(th5_1), std::real(th6_1),
        std::real(th1_1), std::real(th2_2), std::real(th3_2), std::real(th4_2), std::real(th5_2), std::real(th6_2),
        std::real(th1_2), std::real(th2_3), std::real(th3_3), std::real(th4_3), std::real(th5_3), std::real(th6_3),
        std::real(th1_2), std::real(th2_4), std::real(th3_4), std::real(th4_4), std::real(th5_4), std::real(th6_4),
        std::real(th1_1), std::real(th2_5), std::real(th3_5), std::real(th4_5), std::real(th5_1), std::real(th6_1),
        std::real(th1_1), std::real(th2_6), std::real(th3_6), std::real(th4_6), std::real(th5_2), std::real(th6_2),
        std::real(th1_2), std::real(th2_7), std::real(th3_7), std::real(th4_7), std::real(th5_3), std::real(th6_3),
        std::real(th1_2), std::real(th2_8), std::real(th3_8), std::real(th4_8), std::real(th5_4), std::real(th6_4);

    return ret;
}
//-----------------------------------------------//


// compute the jacobian matrix
Eigen::MatrixXf jacobian(Eigen::VectorXf q)
{
    Eigen::VectorXf A(6);
    A << 0, -0.425, -0.3922, 0, 0, 0;
    Eigen::VectorXf D(6);
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;

    Eigen::MatrixXf J(6, 6);
    J.setZero();
    Eigen::MatrixXf J1(6, 1);
    J1 << D(4) * (cos(q(0)) * cos(q(4)) + cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4))) + D(2) * cos(q(0)) + D(3) * cos(q(0)) - A(2) * cos(q(1) + q(2)) * sin(q(0)) - A(1) * cos(q(1)) * sin(q(0)) - D(4) * sin(q(1) + q(2) + q(3)) * sin(q(0)),
        D(4) * (cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4))) + D(2) * sin(q(0)) + D(3) * sin(q(0)) + A(2) * cos(q(1) + q(2)) * cos(q(0)) + A(1) * cos(q(0)) * cos(q(1)) - D(4) * sin(q(1) + q(2) + q(3)) * cos(q(1)),
        0,
        0,
        0,
        1;
    Eigen::MatrixXf J2(6, 1);
    J2 << -cos(q(0)) * (A(2) * sin(q(1) + q(2)) + A(1) * sin(q(1)) + D(4) * (sin(q(1) + q(2)) * sin(q(3)) - cos(q(1) + q(2)) * cos(q(3))) - D(4) * sin(q(4)) * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3)))),
        -sin(q(0)) * (A(2) * sin(q(1) + q(2)) + A(1) * sin(q(1)) + D(4) * (sin(q(1) + q(2)) * sin(q(3)) - cos(q(1) + q(2)) * cos(q(3))) - D(4) * sin(q(4)) * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3)))),
        A(2) * cos(q(1) + q(2)) - (D(4) * sin(q(1) + q(2) + q(3) + q(4))) / 2 + A(1) * cos(q(1)) + (D(4) * sin(q(1) + q(2) + q(3) - q(4))) / 2 + D(4) * sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0;
    Eigen::MatrixXf J3(6, 1);
    J3 << cos(q(0))*(D(4)*cos(q(1) + q(2) + q(3)) - A(2)*sin(q(1)+q(2)) + D(4)*sin(q(1)+q(2)+q(3))*sin(q(4))),
        sin(q(0))*(D(4)*cos(q(1) + q(2) + q(3)) - A(2)*sin(q(1)+q(2)) + D(4)*sin(q(1)+q(2)+q(3))*sin(q(4))),
        A(2)*cos(q(1) + q(2)) - (D(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + (D(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + D(4)*sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0;
    Eigen::MatrixXf J4(6, 1);
    J4 << D(4)*cos(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
          D(4)*sin(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
        D(4)*(sin(q(1) +q(2) + q(3) -q(4))/2 + sin(q(1) + q(2) +q(3)) - sin(q(1) + q(2) + q(3) + q(4))/2),
        sin(q(0)),
        -cos(q(0)),
        0;
    Eigen::MatrixXf J5(6, 1);
    J5 << -D(4)*sin(q(0))*sin(q(4)) - D(4)*cos(q(1)+q(2)+q(3))*cos(q(0))*cos(q(4)),
        D(4)*cos(q(0))*sin(q(4)) - D(4)*cos(q(1)+q(2)+q(3))*cos(q(4))*sin(q(0)),
        -D(4)*(sin(q(1)+q(2)+q(3))/2 + sin(q(1)+q(2)+q(3)+q(4))/2),
        sin(q(1)+q(2)+q(3))*cos(q(0)),
        sin(q(1)+q(2)+q(3))*sin(q(0)),
        -cos(q(1)+q(2)+q(3));
    Eigen::MatrixXf J6(6, 1);
    J6 << 0,
        0,
        0,
        cos(q(4))*sin(q(0)) - cos(q(1)+q(2)+q(3))*cos(q(0))*sin(q(4)),
        -cos(q(0))*cos(q(4)) + cos(q(1)+q(2)+q(3))*sin(q(0))*sin(q(4)),
        -sin(q(1)+q(2)+q(3))*sin(q(4));
    J << J1, J2, J3, J4, J5, J6;
    return J;
}

Eigen::Vector3f ve(float t){
    float omega = 1;
    Eigen::Vector3f v;
    v << t,0,0;//0.8*omega*cos(omega*t), -0.8*omega*sin(omega*t), 0 ;
    return v;
}

Eigen::MatrixXf invDiffKinematics (Eigen::VectorXf TH0, float minT, float maxT, float Dt)
{
    Eigen::VectorXf T(1);
    for (float i = minT; i < maxT; i += Dt)
    {
        T(T.rows()-1) = i;
        T.conservativeResize(T.rows() + 1);
    }
    int L = T.size();
    Eigen::VectorXf qk = TH0;
    Eigen::MatrixXf q = qk.transpose();
    Eigen::MatrixXf Jac (6,6);
    Eigen::VectorXf dotqk(6); 
    Eigen::Vector3f omegae;
    omegae << 0, 0, 0;
    Eigen::VectorXf aux (6);
    Eigen::VectorXf qk1 (6);

    for(int i=0 ; i<L ; i++)
    {
        Jac = jacobian(qk);
        aux << ve(i), omegae;
        dotqk = Jac.inverse() * aux;
        qk1 = qk + dotqk * Dt; 
        q.conservativeResize(q.rows()+1, q.cols());
        q.block(q.rows()-1, 0, 1, 6) = qk1.transpose();
        qk = qk1;
    }
    return q;
}

Eigen::VectorXf invDiffKinematicControlComplete(Eigen::VectorXf q, Eigen::Vector3f xe, Eigen::Vector3f xd, Eigen::Vector3f vd, Eigen::Vector3f phie, Eigen::Vector3f phid, Eigen::Vector3f phiddot, Eigen::Matrix3f kp, Eigen::Matrix3f kphi){
    Eigen::MatrixXf J;
    J = jacobian(q);
    float alpha = phie(2);
    float beta = phie(1);
    float gamma = phie(0);
    Eigen::Matrix3f T;
    T << cos(beta)*cos(gamma), -sin(gamma), 0,
        cos(beta)*sin(gamma), cos(gamma), 0,
        -sin(beta), 0, 1;
    Eigen::MatrixXf Ta(6,6);
    Ta << Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(),
        Eigen::Matrix3f::Zero(), T;
    Eigen::MatrixXf Ja(6,6);
    Ja << Ta.inverse()*J;
    Eigen::VectorXf qdot;
    Eigen::VectorXf aux(6);
    aux << vd+kp*(xd-xe), phiddot+kphi*(phid-phie);
    qdot = Ja.inverse()* aux;
    return qdot;
}

Eigen::Vector3f pd(float t, Eigen::Vector3f xef, Eigen::Vector3f xe0){
    if(t>1){
        return xef;
    }
    else{
        return t*xef+(1-t)*xe0;
    }
}

Eigen::Vector3f phid(float t, Eigen::Vector3f phief, Eigen::Vector3f phie0){
    if(t>1){
        return phief;
    }
    else{
        return t*phief+(1-t)*phie0;
    }
}

Eigen::MatrixXf invDiffKinematicControlSimComplete(Eigen::VectorXf TH0, Eigen::Vector3f xef, Eigen::Vector3f phief, Eigen::Matrix3f kp, Eigen::Matrix3f kphi, float minT, float maxT, float dt){
    Eigen::VectorXf T(1);
    dkin start;
    for (float i = minT; i < maxT; i += dt)
    {
        T(T.rows()-1) = i;
        T.conservativeResize(T.rows() + 1);
    }
    int L = T.rows();
    Eigen::VectorXf qk = TH0;
    Eigen::MatrixXf q = qk.transpose();
    Eigen::MatrixXf Jac (6,6);
    Eigen::VectorXf dotqk(6); 
    Eigen::Vector3f omegae;
    omegae << 0, 0, 0;
    Eigen::VectorXf aux (6);
    Eigen::VectorXf qk1 (6);
    Eigen::Vector3f phie;
    Eigen::Vector3f vd;
    Eigen::Vector3f phiddot;
    for(int i = 0; i < L-2; i++){
        start = directKin(qk);
        phie = start.rot.eulerAngles(0,1,2);
        vd = (pd(i, xef, start.xyz)-pd(i-dt, xef, start.xyz))/dt;
        phiddot = (phid(i, phief, phie)-phid(i-dt, phief, phie))/dt;
        dotqk = invDiffKinematicControlComplete(qk, start.xyz, pd(i, xef, start.xyz), vd, phie.transpose().conjugate(), phid(i, phief, phie), phiddot, kp, kphi);
        qk1 = qk + dotqk * dt;
        q.conservativeResize(q.rows()+1, q.cols());
        q.block(q.rows()-1, 0, 1, 6) = qk1.transpose();
        qk = qk1;
    }
    return q;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(1000);

    std_msgs::Float64MultiArray jointState_msg_robot;
    jointState_msg_robot.data.resize(9);
    
    Eigen::VectorXf TH0(6);
    Eigen::MatrixXf q;
    Eigen::Vector3f xef;
    Eigen::Vector3f phief;
    Eigen::Matrix3f kp;
    Eigen::Matrix3f kphi;
    float minT;
    float maxT;
    float dt;
    kp << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    kphi << 0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1;

    minT = 0;
    maxT = 1;
    dt = 0.01;
    TH0 <<  -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017;
    float error = 1000;
    while(true){
        //read from user input xef and phief
        std::cout << "Enter xef: ";
        std::cin >> xef(0) >> xef(1) >> xef(2);
        std::cout << std::endl;

        std::cout << "Enter phief: ";
        std::cin >> phief(0) >> phief(1) >> phief(2);
        dkin now;
        
        do{
            q = invDiffKinematicControlSimComplete(TH0, xef, phief, kp, kphi, minT, maxT, dt);
            // publish qs to robot
            for (int i = 0; i < q.rows(); i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    jointState_msg_robot.data[j] = q(i, j);
                }
                jointState_msg_robot.data[6] = 0;
                jointState_msg_robot.data[7] = 0;
                jointState_msg_robot.data[8] = 0;

                pub_des_jstate.publish(jointState_msg_robot);
                ros::spinOnce();
                loop_rate.sleep();
            }
            TH0 = q.row(q.rows()-1);
            //caluculate error
            now = directKin(TH0);
            error = (now.xyz-xef).norm() + (now.rot.eulerAngles(0,1,2)-phief).norm();
            std::cout << "Error: " << error << std::endl;
        }while(error > 0.05);
    }
    

    return 0;
}