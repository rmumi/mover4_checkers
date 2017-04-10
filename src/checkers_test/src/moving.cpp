#include "moving.h"

void joint_states_callback(const sensor_msgs::JointState &msg) {
     std::cout << msg.position[0] << std::endl;
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "moving");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 10, joint_states_callback);

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("CPRMoverJointVel", 20);

    ros::Rate loop_rate(20);

    while(ros::ok()) {
        sensor_msgs::JointState pub_vel;
        pub_vel.header.stamp = ros::Time::now();
        pub_vel.velocity.resize(4);
        pub_vel.name.resize(4);
        for(int i = 0; i < 4; i++) {
            pub_vel.velocity[i] = 5.0;
            pub_vel.name[i] = (std::string("Joint") + std::to_string(i+1)).c_str();
        }
        pub.publish(pub_vel);

        ros::spinOnce();  // process callbacks

        loop_rate.sleep();
    }

    return 0;
}