#include "moving.h"

const double PI = 3.14159265359;
const double deg2rad = PI / 180.0;
const double rad2deg = 180.0 / PI;
robotState robot_current;

void joint_states_callback(const sensor_msgs::JointState &msg) {
     for(int i = 0; i < 4; i++) {
         robot_current.j[i] = msg.position[i];
         std::cout << msg.position[i] << std::endl;
     }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "moving");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 10, joint_states_callback);

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("CPRMoverJointVel", 20);

    ros::Rate loop_rate(20);

    robotState requested;
    requested.j[0] = 48 * deg2rad;
    requested.j[1] = 33 * deg2rad;
    requested.j[2] = 119 * deg2rad;
    requested.j[3] = 10 * deg2rad;

    checkers::Matrix<double> matr(4);
    // matr.Transpose();
    // checkers::HTMatrix<double> matr_2(matr);
    // matr_2.Inverse();

    while(ros::ok()) {
        sensor_msgs::JointState pub_vel;
        pub_vel.header.stamp = ros::Time::now();
        pub_vel.velocity.resize(4);
        pub_vel.name.resize(4);
        for(int i = 0; i < 4; i++) {
            pub_vel.velocity[i] = std::min(180 * (requested.j[i] - robot_current.j[i]), 95.0);
            pub_vel.name[i] = (std::string("Joint") + std::to_string(i+1)).c_str();
        }
        pub.publish(pub_vel);

        ros::spinOnce();  // process callbacks

        loop_rate.sleep();
    }

    return 0;
}
