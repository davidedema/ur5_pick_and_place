#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <sstream>
#include <cmath>
#include <complex.h>

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

Eigen::Vector3f xe(float t, dkin &frame, float x, float y, float z)
{
    Eigen::Vector3f xef;
    xef << x, y, z;
    return t * xef + (1 - t) * frame.xyz;
}

Eigen::Vector3f phie(float t, Eigen::Vector3f phie0, float r, float p, float y)
{
    Eigen::Vector3f phief;
    phief << r, p, y;
    return t * phief + (1 - t) * phie0;
}

int main(int argc, char **argv)
{
    // make a publisher node called "custom_joint_publisher" for a 9DoF robot
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(1000);
    Eigen::MatrixXf th(1, 6);
    Eigen::MatrixXf TH(8, 6);
    Eigen::VectorXf th0(6);
    Eigen::Vector3f phie0;
    dkin frame;
    float xi, y, z, r, p, yw;
    int selected;
    Eigen::VectorXf min(6);
    float dist, min_dist;
    bool use = false;
    std_msgs::Float64MultiArray jointState_msg_robot;
    jointState_msg_robot.data.resize(9);
    th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017;
    while(ros::ok()){
        //read x y z r p y from user input
        std::cout << "Enter x y z: ";
        std::cin >> xi >> y >> z;
        std::cout << "Enter r p y: ";
        std::cin >> r >> p >> yw;
        

        
        frame = directKin(th0);
        phie0 = frame.rot.eulerAngles(0, 1, 2);
        selected = 0;        
        min_dist = 1000;
        
        
        for (float t = 0; t <= 1; t += 0.01)
        {
            Eigen::Vector3f x = xe(t, frame, xi, y, z);
            Eigen::Vector3f phi = phie(t, phie0, r, p, yw);
            dkin frame2;
            frame2.xyz = x;
            Eigen::Matrix3f Rf;
            Rf = Eigen::AngleAxisf(phi(0), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(phi(2), Eigen::Vector3f::UnitX());
            frame2.rot = Rf;
            TH = invKin(frame2);

            std::cout << "-----------------------" << std::endl;

            std::cout << TH << std::endl;

            std::cout << "------------------------" << std::endl;

            th.conservativeResize(th.rows() + 1, th.cols());
            // nan check
            for (int i = 0; i < 8; i++)
            {
                use = true;
                for (int j = 0; j < 6; j++)
                {
                    if (isnan(TH(i, j)))
                    {
                        use = false;
                        break;
                    }
                }
                // if valid row
                if (use)
                {
                    // calc distance from current joint angles to desired joint angles
                    if (th.rows() == 1)
                        min = TH.row(i) - th.row(th.rows() - 1);
                    else
                        min = TH.row(i) - th.row(th.rows() - 2);
                    for (int j = 0; j < 6; j++)
                    {
                        dist += std::abs(min(j));
                    }
                    dist /= 6;
                    // if distance is less than current min distance
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        selected = i;
                    }
                    th.row(th.rows() - 1) = TH.row(selected);
                    for (int j = 0; j < 6; j++)
                    {
                        jointState_msg_robot.data[j] = TH(selected, j);
                    }
                    jointState_msg_robot.data[6] = 0;
                    jointState_msg_robot.data[7] = 0;
                    jointState_msg_robot.data[8] = 0;

                    pub_des_jstate.publish(jointState_msg_robot);

                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
        }
        th0 = th.row(th.rows() - 1);
    }
    
    ros::Duration(1.0).sleep();

    return 0;
}
