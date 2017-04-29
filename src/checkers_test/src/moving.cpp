#include "moving.h"

robotState robot_current;

void joint_states_callback(const sensor_msgs::JointState &msg) {
     for(int i = 0; i < 4; i++) {
         robot_current.j[i] = msg.position[i];
         std::cout << msg.position[i] << std::endl;
     }
}

robotState fkine(const robotState &rb) {

}
// TODO check if the position is reachable
robotState InvKine(const robotState &rb, int way=0) {  // way = {0 - ellbow-up, 1 - ellbow-down, 2 - turned ellbow-up, 3 - turned ellbow-down}
    double phi = rb.p[3];
    double xr = rb.p[0], yr = rb.p[1], zr = rb.p[2];
    double xm = zr - a0;
    double ym = ((way&2)?-1:1) * sqrt(xr*xr + yr*yr);
    double th0 = atan2(yr, xr);
    if(way&2) {
        th0 = PI + th0;
        if(th0 > 2*PI - EPS) th0 -= 2*PI;
    }
    double xw = xm - a3 * cos(phi);
    double yw = ym - a3 * sin(phi);
    double c2 = (xw*xw + yw*yw - a1*a1 - a2*a2) / (2*a1*a2);
    double s2 = ((way&1)?-1:1) * sqrt(1 - c2*c2);
    double th2 = atan2(s2, c2);
    double th1 = atan2((a1 + a2*c2)*yw - a2*s2*xw,
                       (a1 + a2*c2)*xw + a2*s2*yw);
    double th3 = phi - th1 - th2;
    robotState ret(rb);
    ret.j[0] = th0; ret.j[1] = th1;
    ret.j[2] = th2; ret.j[3] = th3;
    return ret;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "moving");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 10, joint_states_callback);

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("CPRMoverJointVel", 20);

    ros::Rate loop_rate(20);

    robotState requested;
    // requested.j[0] = 48 * deg2rad;
    // requested.j[1] = 33 * deg2rad;
    // requested.j[2] = 119 * deg2rad;
    // requested.j[3] = 10 * deg2rad;
    requested.p[0] = 200;
    requested.p[1] = 200;
    requested.p[2] = 200;
    requested.p[3] = PI;
    requested = InvKine(requested, 0);
    std::cout << "Uglovi:" << requested.j[0] << requested.j[1] << requested.j[2] << requested.j[3] << std::endl;

    Matrix matr(4);
    matr.Transpose();
    HTMatrix matr_2(matr);
    matr_2.Inverse();

    while(ros::ok()) {
        sensor_msgs::JointState pub_vel;
        std::cout << "Uglovi:\t" << requested.j[0] * rad2deg << "\t" << requested.j[1] * rad2deg << "\t" << requested.j[2] * rad2deg<< "\t" << requested.j[3]* rad2deg << std::endl;
        std::cout << "Trenut:\t" << robot_current.j[0] * rad2deg << "\t" << robot_current.j[1] * rad2deg << "\t" << robot_current.j[2] * rad2deg<< "\t" << robot_current.j[3] * rad2deg<< std::endl;
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
