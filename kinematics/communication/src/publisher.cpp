#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <sstream>
#include <cmath>

struct dkin
{
    Eigen::Matrix3f rot;
    Eigen::Vector3f xyz;
};

Eigen::Matrix4f t10f(float th1){
    Eigen::Matrix4f aux;
    //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << cos(th1), -sin(th1), 0, 0,
           sin(th1), cos(th1), 0, 0,
           0, 0, 1, d(0),
           0, 0, 0, 1;
    return aux;
}

Eigen::Matrix4f t21f(float th2){
    Eigen::Matrix4f aux;
    //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << cos(th2), -sin(th2), 0, 0,
           0, 0, -1, 0,
           sin(th2), cos(th2), 0, 0,
           0, 0, 0, 1;
    return aux;
}

Eigen::Matrix4f t32f(float th3){
    Eigen::Matrix4f aux;
    //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << cos(th3), -sin(th3), 0, a(1),
           sin(th3), cos(th3), 0, 0,
           0, 0, 1, d(2),  
           0, 0, 0, 1;
    return aux; 
}

Eigen::Matrix4f t43f(float th4){
    Eigen::Matrix4f aux;
    //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << cos(th4), -sin(th4), 0, a(2),
           sin(th4), cos(th4), 0, 0,
           0, 0, 1, d(3),
           0, 0, 0, 1;
    return aux;
}

Eigen::Matrix4f t54f(float th5){
    Eigen::Matrix4f aux;
     //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << cos(th5), -sin(th5), 0, 0,
           0, 0, -1, -d(4),
           sin(th5), cos(th5), 0, 0,
           0, 0, 0, 1;
    return aux;
}

Eigen::Matrix4f t65f(float th6){
    Eigen::Matrix4f aux;
     //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    aux << cos(th6), -sin(th6), 0, 0,
           0, 0, 1, d(5),
           -sin(th6), -cos(th6), 0, 0,
           0, 0, 0, 1;
    return aux;
}


/**
 * @brief compute the direct kinematics
 * 
 * @param th joint angles
 * @return dkin : rot matrix and cartesian point (respect of the base frame i think)
 */
dkin directKin(Eigen::VectorXf th){
    dkin ret;
    //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;

    Eigen::Matrix4f t60 = t10f(th(0))*t21f(th(1))*t32f(th(2))*t43f(th(3))*t54f(th(4))*t65f(th(5));

    ret.rot = t60.block(0,0,3,3);
    ret.xyz = t60.block(0,3,3,1);

    return ret;
}

Eigen::MatrixXf invKin(dkin &frame){
    //Vector of A distances (meters)
    Eigen::VectorXf a(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    //Vector of D distances (meters)
    Eigen::VectorXf d(6);
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;

    Eigen::Matrix4f t60;

    // Matrix creation

    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            t60(i,j) = frame.rot(i, j);        
        } 
    }

    for(int i=0; i<3; i++){
        t60(i,3) = frame.xyz(i);
    }

    for(int j=0; j<3; j++){
        t60(3,j) = 0;
    }

    t60(3,3) = 1;

    // finding th1

    Eigen::Vector4f aux;
    aux << 0, 0, -d(5), 1;
    Eigen::Vector4f p50;
    p50 = t60*aux;
    float th1_1 = std::real(atan2(p50(1), p50(0)) + acos(d(3)/hypot(p50(1), p50(0))))+ M_PI/2;
    float th1_2 = std::real(atan2(p50(1), p50(0)) - acos(d(3)/hypot(p50(1), p50(0))))+ M_PI/2;

    // finding th5

    float th5_1 = +std::real(acos((frame.xyz(0)*sin(th1_1) - frame.xyz(1)*cos(th1_1)-d(3)) / d(5)));
    float th5_2 = -std::real(acos((frame.xyz(0)*sin(th1_1) - frame.xyz(1)*cos(th1_1)-d(3)) / d(5)));
    float th5_3 = +std::real(acos((frame.xyz(0)*sin(th1_2) - frame.xyz(1)*cos(th1_2)-d(3)) / d(5)));
    float th5_4 = -std::real(acos((frame.xyz(0)*sin(th1_2) - frame.xyz(1)*cos(th1_2)-d(3)) / d(5)));

    // related to th11 a th51
    Eigen::Matrix4f t06;
    t06 = t60.inverse();
    Eigen::Vector3f Xhat;
    Xhat = t06.block(0,0,3,1);
    Eigen::Vector3f Yhat;
    Yhat = t06.block(0,1,3,1);

    float th6_1 = std::real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1)));
    
    // related to th11 a th52
    float th6_2 = std::real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_2), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_2)));

    // related to th12 a th53
    float th6_3 = std::real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_3), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_3)));

    // related to th12 a th54
    float th6_4 = std::real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_4), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_4)));

    Eigen::Matrix4f t41m;
    Eigen::Vector3f p41_1;
    Eigen::Vector3f p41_2;
    Eigen::Vector3f p41_3;
    Eigen::Vector3f p41_4;
    float p41xz_1;
    float p41xz_2;
    float p41xz_3;
    float p41xz_4;

    t41m = t10f(th1_1).inverse()*t60*t65f(th6_1).inverse()*t54f(th5_1).inverse();
    p41_1 = t41m.block(0,3,3,1);
    p41xz_1 = hypot(p41_1(0), p41_1(2));

    t41m = t10f(th1_1).inverse()*t60*t65f(th6_2).inverse()*t54f(th5_2).inverse();
    p41_2 = t41m.block(0,3,3,1);
    p41xz_2 = hypot(p41_2(0), p41_2(2));

    t41m = t10f(th1_2).inverse()*t60*t65f(th6_3).inverse()*t54f(th5_3).inverse();
    p41_3 = t41m.block(0,3,3,1);
    p41xz_3 = hypot(p41_3(0), p41_3(2));

    t41m = t10f(th1_2).inverse()*t60*t65f(th6_4).inverse()*t54f(th5_4).inverse();
    p41_4 = t41m.block(0,3,3,1);
    p41xz_4 = hypot(p41_4(0), p41_4(2));

    // computation of eight possible value for th3
    float th3_1 = std::real(acos((pow(p41xz_1,2)-pow(a(1),2)-pow(a(2),2))/(2*a(1)*a(2))));
    float th3_2 = std::real(acos((pow(p41xz_2,2)-pow(a(1),2)-pow(a(2),2))/(2*a(1)*a(2))));
    float th3_3 = std::real(acos((pow(p41xz_3,2)-pow(a(1),2)-pow(a(2),2))/(2*a(1)*a(2))));
    float th3_4 = std::real(acos((pow(p41xz_4,2)-pow(a(1),2)-pow(a(2),2))/(2*a(1)*a(2))));

    float th3_5 = -th3_1;
    float th3_6 = -th3_2;
    float th3_7 = -th3_3;
    float th3_8 = -th3_4;

    // computation of eight possible value for th2
    float th2_1 = std::real(atan2(-p41_1(2), -p41_1(0)) - asin((-a(2)*sin(th3_1))/p41xz_1));
    float th2_2 = std::real(atan2(-p41_2(2), -p41_2(0)) - asin((-a(2)*sin(th3_2))/p41xz_2));
    float th2_3 = std::real(atan2(-p41_3(2), -p41_3(0)) - asin((-a(2)*sin(th3_3))/p41xz_3));
    float th2_4 = std::real(atan2(-p41_4(2), -p41_4(0)) - asin((-a(2)*sin(th3_4))/p41xz_4));

    float th2_5 = std::real(atan2(-p41_1(2), -p41_1(0)) - asin((a(2)*sin(th3_1))/p41xz_1));
    float th2_6 = std::real(atan2(-p41_2(2), -p41_2(0)) - asin((a(2)*sin(th3_2))/p41xz_2));
    float th2_7 = std::real(atan2(-p41_3(2), -p41_3(0)) - asin((a(2)*sin(th3_3))/p41xz_3));
    float th2_8 = std::real(atan2(-p41_4(2), -p41_4(0)) - asin((a(2)*sin(th3_4))/p41xz_4));

    Eigen::Matrix4f t43m;
    Eigen::Vector3f xhat43; 
    t43m = t32f(th3_1).inverse()*t21f(th2_1).inverse()*t10f(th1_1).inverse()*t60*t65f(th6_1).inverse()*t54f(th5_1).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_1 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_2).inverse()*t21f(th2_2).inverse()*t10f(th1_1).inverse()*t60*t65f(th6_2).inverse()*t54f(th5_2).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_2 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_3).inverse()*t21f(th2_3).inverse()*t10f(th1_2).inverse()*t60*t65f(th6_3).inverse()*t54f(th5_3).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_3 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_4).inverse()*t21f(th2_4).inverse()*t10f(th1_2).inverse()*t60*t65f(th6_4).inverse()*t54f(th5_4).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_4 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_5).inverse()*t21f(th2_5).inverse()*t10f(th1_1).inverse()*t60*t65f(th6_1).inverse()*t54f(th5_1).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_5 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_6).inverse()*t21f(th2_6).inverse()*t10f(th1_1).inverse()*t60*t65f(th6_2).inverse()*t54f(th5_2).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_6 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_7).inverse()*t21f(th2_7).inverse()*t10f(th1_2).inverse()*t60*t65f(th6_3).inverse()*t54f(th5_3).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_7 = std::real(atan2(xhat43(1), xhat43(0)));

    t43m = t32f(th3_8).inverse()*t21f(th2_8).inverse()*t10f(th1_2).inverse()*t60*t65f(th6_4).inverse()*t54f(th5_4).inverse();
    xhat43 = t43m.block(0,0,3,1);
    float th4_8 = std::real(atan2(xhat43(1), xhat43(0)));

    Eigen::MatrixXf ret(8,6);
    ret << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
           th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
           th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
           th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
           th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
           th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
           th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
           th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;
    
    return ret;

} 

Eigen::Vector3f xe(float t, dkin &frame){
    Eigen::Vector3f xef;
    xef << -0.8173, 0.1915, 0.0055;
    return t*xef+(1-t)*frame.xyz;
}

Eigen::Vector3f phie(float t, Eigen::Vector3f phie0){
    Eigen::Vector3f phief;
    phief << M_PI, M_PI/4, 3*M_PI/4;
    return t*phief+(1-t)*phie0;
}



int main(int argc, char **argv)
{
    // make a publisher node called "custom_joint_publisher" for a 9DoF robot
    /* ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(1000);

    Eigen::VectorXf th0(6);
    th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017;
    dkin frame = directKin(th0);
    Eigen::Vector3f phie0 = frame.rot.eulerAngles(0,1,2);
    Eigen::MatrixXf th(1,6);
    Eigen::MatrixXf TH(8,6);
    bool use = false;
    for (float t = 0; t <= 1; t += 0.01){
        Eigen::Vector3f x = xe(t, frame);
        Eigen::Vector3f phi = phie(t, phie0);
        dkin frame2;
        frame2.xyz = x;
        Eigen::Matrix3f Rf;
        Rf = Eigen::AngleAxisf(phi(0), Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(phi(1), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(phi(2), Eigen::Vector3f::UnitX());
        frame2.rot = Rf;
        TH = invKin(frame2);
        
        std::cout << "-----------------------" << std::endl;
        
        std::cout << TH << std::endl;

        std::cout << "------------------------" << std::endl;
        
        th.conservativeResize(th.rows()+1, th.cols());
        //nan check
        for(int i = 0; i<8; i++){
            use = true;
            for(int j = 0; j<6; j++){
                if(isnan(TH(i,j))){
                    use = false;
                    break;
                }   
            }
            if(use){
                th.row(th.rows()-1) = TH.row(i);
                use = false;
                break;
            }
        } 
    }

    std::cout << th << std::endl;

    Eigen::VectorXf q(6);
    q << -0.1, -0.1, -0.1, -0.1, -0.1, -0.1;

    dkin frame = directKin(q);

    std::cout << frame.xyz << std::endl;
    
    // compute inv kin for point (0.2, 0.5, -0.5)
    //frame.xyz << 0.2, 0.5, -0.5;
    Eigen::MatrixXf inv = invKin(frame);

    // print inv kin values
    std::cout << "inv " << std::endl << inv << std::endl; 
    
    // create a message to send to the robot
    std_msgs::Float64MultiArray jointState_msg_robot;
    jointState_msg_robot.data.resize(9);

    // send the message to the robot
    pub_des_jstate.publish(jointState_msg_robot);

    // wait for 1 second
    ros::Duration(1.0).sleep(); 

    // read values from user input and send to robot
    while (ros::ok())
    {
        for(int i = 0; i < th.rows(); i++){
            for(int j = 0; j < 6; j++){
                jointState_msg_robot.data[j] = th(i,j);
            }
            jointState_msg_robot.data[6] = 0.0;
            jointState_msg_robot.data[7] = 0.0;
            jointState_msg_robot.data[8] = 0.0;                jointState_msg_robot.data[j] = th(i,j);

            pub_des_jstate.publish(jointState_msg_robot);
            ros::spinOnce();
            loop_rate.sleep();
        }
    } */

    /* Eigen::VectorXf th0(6);
    th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017;*/
    dkin frame; 
    frame.xyz << -0.8173, 0.1915, 0.0055;
    frame.rot << 0, 0, 0,
                 0, 0, 0, 
                 0, 0, 0;
    Eigen::MatrixXf inv = invKin(frame);

    std::cout << inv << std::endl;

    return 0;
}
