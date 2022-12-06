
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "kinematics.h"
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"

Eigen::MatrixXf p2pMotionPlan(frame start, frame goal, float minT, float maxT, float dt)
{
    Eigen::MatrixXf Q0 = invKin(start);
    Eigen::MatrixXf Qf = invKin(goal);

    Eigen::VectorXf q0(6);
    q0 = Q0.row(0).transpose();
    Eigen::VectorXf qf(6);
    qf = Qf.row(0).transpose();

    Eigen::Matrix4f M;
    Eigen::Vector4f b;
    Eigen::Vector4f a;
    Eigen::MatrixXf A(1, 4);


    for (int i = 0; i < 6; i++)
    {
        M << 1, minT, pow(minT, 2), pow(minT, 3),
            0, 1, 2 * minT, 3 * pow(minT, 2),
            1, maxT, pow(maxT, 2), pow(maxT, 3),
            0, 1, 2 * maxT, 3 * pow(maxT, 2);
        b << q0(i), 0, qf(i), 0;
        a = M.inverse() * b;
        A.row(A.rows() - 1) = a.transpose();
        A.conservativeResize(A.rows() + 1, A.cols());
    }
    Eigen::MatrixXf TH(1, 6);
    Eigen::MatrixXf xE(1, 4);
    Eigen::MatrixXf phiE(1, 4);
    Eigen::VectorXf q(6);
    frame aux;

    for (int i = minT; i < maxT; i += dt)
    {
        for (int j = 0; j < 6; j++)
        {
            q(j) = A(j, 0) + A(j, 1) * i + A(j, 2) * pow(i, 2) + A(j, 3) * pow(i, 3);
        }
        TH.row(TH.rows() - 1) = q.transpose();
        TH.conservativeResize(TH.rows() + 1, TH.cols());
        aux = directKin(q);
    }
    return TH;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(1000);

    std_msgs::Float64MultiArray jointState_msg_robot;
    jointState_msg_robot.data.resize(9);

    frame start;
    Eigen::VectorXf th0(6);
    th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017;
    start = directKin(th0);

    frame goal;
    goal.xyz << 0.5, -0.5, 0.5;
    goal.rot = Eigen::Matrix3f::Identity();

    Eigen::MatrixXf TH;
    TH = p2pMotionPlan(start, goal, 0, 100, 1);

    for (int i = 0; i < TH.rows(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            jointState_msg_robot.data[j] = TH(i, j);
        }
        jointState_msg_robot.data[6] = 0;
        jointState_msg_robot.data[7] = 0;
        jointState_msg_robot.data[8] = 0;

        pub_des_jstate.publish(jointState_msg_robot);
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}