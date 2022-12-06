#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <complex>

struct frame
{
    Eigen::Matrix3f rot;
    Eigen::Vector3f xyz;
};

//submatrix of the transformation matrix
/**
 * @brief create the transformation matrix for the first joint
 * 
 * @param th1 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f t10f(std::complex<float> th1);

/**
 * @brief create the transformation matrix for the second joint
 * 
 * @param th2 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f t21f(std::complex<float> th2);

/**
 * @brief create the transformation matrix for the third joint
 * 
 * @param th3 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f t32f(std::complex<float> th3);

/**
 * @brief create the transformation matrix for the fourth joint
 * 
 * @param th4 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f t43f(std::complex<float> th4);

/**
 * @brief create the transformation matrix for the fifth joint
 * 
 * @param th5 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f t54f(std::complex<float> th5);

/**
 * @brief create the transformation matrix for the sixth joint
 * 
 * @param th6 
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f t65f(std::complex<float> th6);

/**
 * @brief compute the direct kinematics
 *
 * @param th joint angles
 * @return frame : rot matrix and cartesian point respect of the base frame
 */
frame directKin(Eigen::VectorXf th);

/**
 * @brief compute the inverse kinematics
 *
 * @param rot desired orientation of the end effector
 * @param xyz desired position of the end effector
 * @return ikin : joint angles
 */
Eigen::MatrixXf invKin(frame &frame);

/**
 * @brief Calculate the jacobian matrix
 * 
 * @param q joint angles
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf jacobian(Eigen::VectorXf q);

#endif