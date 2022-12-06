#include "ros/ros.h"
#include "kinematics.h"
#include <std_msgs/Float64MultiArray.h> 
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <complex>

Eigen::VectorXf invDiffKinematicControlComplete(Eigen::VectorXf q, Eigen::Vector3f xe, Eigen::Vector3f xd, Eigen::Vector3f vd, Eigen::Vector3f phie, Eigen::Vector3f phid, Eigen::Vector3f phiddot, Eigen::Matrix3f kp, Eigen::Matrix3f kphi){
    Eigen::MatrixXf J;
    J = jacobian(q);
    float alpha = phie(0); // phie(2);
    float beta = phie(1); // phie(1);
    float gamma = phie(2); // phie(0);
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
    kp <<   1, 0, 0,
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
    float prev_error;
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
            std::cout << "error position: " << (xef - now.xyz).norm() << std::endl;
            std::cout << "error orientation: " << (phief - now.rot.eulerAngles(0,1,2)).norm() << std::endl;
            prev_error = error;
            error = (directKin(TH0).xyz - xef).norm() + (directKin(TH0).rot.eulerAngles(0,1,2) - phief).norm();
            now = directKin(TH0);
        }while(error > 0.07);
    }
    

    return 0;
}