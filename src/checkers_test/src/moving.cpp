#include "moving.h"
#include "trajectory.h"

robotState robot_current;

deque<RobotAction> actions;
Trajectory5 current_trajectory(robot_current, robot_current, 1);
ros::Publisher cpr_commands_pub, robot_state_msg_pub, robot_sig_pub;

void joint_states_callback(const sensor_msgs::JointState &msg) {
     for(int i = 0; i < 4; i++) {
         robot_current.j[i] = msg.position[i];
        //  std::cout << msg.position[i] << "\t";
     }
    //  std::cout << std::endl;
}

void robot_sig_callback(const std_msgs::String &a) {
    if(a.data == "ROBOT_STOP") {
        current_trajectory.Finish();
        actions.clear();
    }
}

void robot_point_msg_callback(const std_msgs::Float64MultiArray &msg) {
    robotState a;
    for(int i = 0; i < 4; i++) {
        a.p[i] = msg.data[i];
    }
    int flags = static_cast<int>(msg.data[4] + 0.01);
    int orient = static_cast<int>(msg.data[5] + 0.01);
    double wait = msg.data[6];
    a = InvKine(a, orient);
    actions.emplace_back(flags, a, orient, wait);
    return;
}

void NextTrajectory() {
    if(!actions.size()) return;
    if(actions.front().gripper_open) {
        std_msgs::String _z;
        _z.data = std::string("GripperOpen");
        cpr_commands_pub.publish(_z);
        actions.pop_front();
    } else if(actions.front().gripper_close) {
        std_msgs::String _z;
        _z.data = std::string("GripperClose");
        cpr_commands_pub.publish(_z);
        actions.pop_front();
    } else if(actions.front().waiting && actions.front().wait > 1) {
        actions.front().wait--;
    } else if(actions.front().waiting && actions.front().wait <= 1) {
        actions.pop_front();
    } else if(actions.front().to_point) {
        current_trajectory = Trajectory5(robot_current,
            actions.front()._q, actions.front().wait/update_f);
        actions.pop_front();
    } else if(actions.front().mid_point) {
        // TODO mid point trajectory6
    } else {
        std::cout << "FAILED ACTION";
        actions.pop_front();
    }
}

robotState ForKine(const robotState &rb) {

}

// TODO check if the position is reachable
robotState InvKine(const robotState &rb, int way=0) {  // way = {0 - ellbow-up, 1 - ellbow-down, 2 - turned ellbow-down, 3 - turned ellbow-up}
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
    if(ret.j[0] != ret.j[0] || ret.j[1] != ret.j[1] || ret.j[2] != ret.j[2] || ret.j[3] != ret.j[3]) {
        for(int i = 0; i < 4; i++) printf("%.3lf\t \n", rb.p[i]);
        std::cout<<"AAAA\n";
    }
    return ret;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "moving");

    ros::NodeHandle n;

    // cpr_mover communication
    ros::Subscriber cpr_joint_states_sub = n.subscribe("/joint_states", 10, joint_states_callback);
    // ros::Publisher cpr_vel_pub = n.advertise<sensor_msgs::JointState>("/CPRMoverJointVel", 10);
    ros::Publisher cpr_pos_pub = n.advertise<sensor_msgs::JointState>("/CPRMoverJointPos", 10);
    cpr_commands_pub = n.advertise<std_msgs::String>("/CPRMoverCommands", 10);

    // checkers communication
    ros::Subscriber point_msg_sub = n.subscribe("/checkers/robot_point_msg", 50, robot_point_msg_callback);
    ros::Subscriber robot_sig_sub = n.subscribe("/checkers/robot_sig", 50, robot_sig_callback);
    robot_state_msg_pub = n.advertise<std_msgs::Float64MultiArray>("/checkers/robot_state_msg", 50);
    robot_sig_pub = n.advertise<std_msgs::String>("/checkers/robot_sig", 50);

    ros::Rate loop_rate(update_f);

    int looped = 0;
    while(ros::ok && looped < 20) {
        ros::spinOnce();  // get init angles
        looped++;
        loop_rate.sleep();
    }

    // init no movement
    actions.clear();
    current_trajectory.Finish();

    // sensor_msgs::JointState pub_vel2;
    // pub_vel2.velocity.resize(4, 0);
    // cpr_vel_pub.publish(pub_vel2);

    robotState requested, req_inter, req_inter2, dummy;
    requested.p[0] = 125;
    requested.p[1] = -102;// -9.75
    requested.p[2] = 50;
    requested.p[3] = PI;

    req_inter.p[0] = 220+150;
    req_inter.p[1] = +90;
    req_inter.p[2] = 50;
    req_inter.p[3] = PI;

    std::swap(req_inter, requested);
    // req_inter2.p[0] = 220;
    // req_inter2.p[1] = 0;
    // req_inter2.p[2] = 150;
    // req_inter2.p[3] = PI;


    req_inter2.p[0] = 200;
    req_inter2.p[1] = 0;
    req_inter2.p[2] = 150;
    req_inter2.p[3] = PI;


    requested = InvKine(requested, 0);
    req_inter = InvKine(req_inter, 0);
    req_inter2 = InvKine(req_inter2, 0);
    std::cout << "Uglovi:" << requested.j[0] << requested.j[1] << requested.j[2] << requested.j[3] << std::endl;

    printf("Trn:\t");
    for(int i = 0; i < 4; i++)
        printf("%lf\t", robot_current.j[i]);
    printf("\n");
    // actions.emplace_back(1, dummy, 0, 4);
    actions.emplace_back(2, dummy, 0, 4);
    actions.emplace_back(16, dummy, 0, 2);
    actions.emplace_back(4, req_inter2, 0, 10);
    actions.emplace_back(4, req_inter, 0, 10);
    actions.emplace_back(1, dummy, 0, 4);
    
    
    // Trajectory6 z(robot_current, requested, req_inter, 7);
    current_trajectory = Trajectory5(robot_current, requested, 10);
    // Matrix matr(4);
    // matr.Transpose();
    // HTMatrix matr_2(matr);
    // matr_2.Inverse();

    printf("Req:\t");
    for(int i = 0; i < 4; i++)
        printf("%lf\t", requested.j[i]);
    printf("\n");

    while(ros::ok()) {

        ros::spinOnce();  // process callbacks, so it's not late

        sensor_msgs::JointState pub_vel;
        // std::cout << "Uglovi:\t" << requested.j[0] * rad2deg << "\t" << requested.j[1] * rad2deg << "\t" << requested.j[2] * rad2deg<< "\t" << requested.j[3]* rad2deg << std::endl;
        // std::cout << "Trenut:\t" << robot_current.j[0] * rad2deg << "\t" << robot_current.j[1] * rad2deg << "\t" << robot_current.j[2] * rad2deg<< "\t" << robot_current.j[3] * rad2deg<< std::endl;
        pub_vel.header.stamp = ros::Time::now();
        // pub_vel.velocity.resize(4);
        pub_vel.position.resize(4);
        pub_vel.name.resize(4);
        if(current_trajectory.IsFinished()) NextTrajectory();
        // auto l = current_trajectory.GetVel(robot_current);
        // for(int i = 0; i < 4; i++) pub_vel.velocity[i] = l[i];
        // cpr_vel_pub.publish(pub_vel);
        auto q = current_trajectory.GetPos(robot_current);
        for(int i = 0; i < 4; i++) pub_vel.position[i] = q[i];
        cpr_pos_pub.publish(pub_vel);


        loop_rate.sleep();
    }

    return 0;
}
