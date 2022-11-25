#include "ros/ros.h"
#include <communication/dest_point.h>
#include <Eigen/Dense>
#include <sstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "position publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise("/posPublisher", 1);
    ros::Rate loop_rate(1000);
    communication::dest_point msg;
    while (ros::ok()){
        //read pos from user input
        std::cout << "Enter the position of the robot: " << std::endl;
        std::cin >> msg.x >> msg.y >> msg.z;
        //read orientation from user input
        std::cout << "Enter the orientation of the robot: " << std::endl;
        std::cin >> msg.roll >> msg.pitch >> msg.yaw;
        //publish the message
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}