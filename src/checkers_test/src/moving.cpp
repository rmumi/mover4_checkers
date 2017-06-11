#include "moving.h"
#include "trajectory.h"

robotState robot_current;

deque<RobotAction> actions;
Trajectory current_trajectory(robot_current, robot_current, 1);
ros::Publisher cpr_commands_pub, robot_state_msg_pub, robot_sig_pub;
bool has_actions;

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
    } else if (a.data == "ROBOT_RESET") {
        std_msgs::String _z;
        _z.data = std::string("Reset");
        cpr_commands_pub.publish(_z);
    } else if (a.data == "ROBOT_ENABLE") {
        std_msgs::String _z;
        _z.data = std::string("Enable");
        cpr_commands_pub.publish(_z);
    } else if (a.data == "ROBOT_DISABLE") {
        std_msgs::String _z;
        _z.data = std::string("Disable");
        cpr_commands_pub.publish(_z);
    } else if (a.data == "ROBOT_CONNECT") {
        std_msgs::String _z;
        _z.data = std::string("Connect");
        cpr_commands_pub.publish(_z);
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
    if(actions.empty()) {
        if(has_actions) {
            has_actions = 0;
            std_msgs::String _z;
            _z.data = std::string("ROBOT_FIN");
            robot_sig_pub.publish(_z);
        }
        return;
    }
    has_actions = 1;
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
        current_trajectory = Trajectory(robot_current,
            actions.front()._q, (double)actions.front().wait/update_f);
        actions.pop_front();
    } else if(actions.front().mid_point) {
        if(actions.size() > 1) {
            RobotAction mid = actions.front();
            actions.pop_front();
            RobotAction fin = actions.front();
            actions.pop_front();
            if(!fin.to_point)
                ROS_INFO("INVALID SEQUANCE OF ACTIONS!");
            // current_trajectory = Trajectory(robot_current, fin._q, mid._q,
            //                                 (double)fin.wait/update_f);
            current_trajectory = Trajectory(robot_current, fin._q, mid._q,
                                        (double)(fin.wait+mid.wait)/update_f,
                                        (double)mid.wait/update_f);
        } else {
            ROS_INFO("TOO SLOW OR FALSE QUEUE!");
            // TODO a false queue could make an inf loop
        }
    } else {
        ROS_INFO("INVALID ACTION!");
        std::cout << "INVALID ACTION";
        actions.pop_front();
    }
}

robotState ForKine(const robotState &rb) {
    HTMatrix A (vector<vector<double>> (
        {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}})
    );
    double th[] = { rb.j[0] + DH_par[0][0],
                    rb.j[1] + DH_par[1][0],
                    rb.j[2] + DH_par[2][0],
                    rb.j[3] + DH_par[3][0]};
    for(int i = 0; i < 4; i++) {
        A *= HTMatrix ({{cos(th[i]), -sin(th[i]), 0, 0},
                        {sin(th[i]), cos(th[i]), 0, 0},
                        {0, 0, 1, DH_par[i][1]},
                        {0, 0, 0, 1}});
        A *= HTMatrix ({{1, 0, 0, DH_par[i][3]},
                        {0, cos(DH_par[i][2]), -sin(DH_par[i][2]), 0},
                        {0, sin(DH_par[i][2]), cos(DH_par[i][2]), 0},
                        {0, 0, 0, 1}});
    }
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            printf("%.2lf\t", A[i][j]);
        }
        printf("\n");
    }
    auto p (A.GetPosVec());
    auto R (A.GetRot());
    robotState z;
    for(int i = 0; i < 3; i++) z.p[i] = p[i];
    z.p[3] = atan2(R[1][2], R[0][2]);
    z.p[4] = atan2(sqrt(R[0][2]*R[0][2] + R[1][2]*R[1][2]), R[2][2]);
    z.p[5] = atan2(R[2][1], -R[2][0]);
    return z;
}

// TODO check if the position is reachable
robotState InvKine(const robotState &rb, int way=0) {  // way = {0 - ellbow-up, 1 - ellbow-down, 2 - turned ellbow-down, 3 - turned ellbow-up}
    double phi = rb.p[3];
    double xr = rb.p[0], yr = rb.p[1], zr = rb.p[2];
    double th0 = atan2(yr, xr);
    // compensate the gripper offsets, suppose the angle is the almost the same
    // double d = sqrt(15*15 + 12*12);
    // double th_s = atan2(15, 12);
    // xr = xr + d * cos(th_s + th0);
    // yr = yr + d * sin(th_s + th0);
    // th0 = atan2(yr, xr);

    // double d = sqrt(14*14 + 11*11);
    // double th_s = atan2(11, 14);
    // xr = xr + d * cos(th_s + th0);
    // yr = yr + d * sin(th_s + th0);
    // th0 = atan2(yr, xr);// + 20./sqrt(xr*xr + yr*yr);


    double xm = zr - a0;
    double ym = ((way&2)?-1:1) * sqrt(xr*xr + yr*yr);
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

void AddActions() {
    robotState a, dummy;
    // actions.emplace_back(1, dummy, 0, 1);

    // a.p[0] = 200;
    // a.p[1] = 0;// -9.75
    // a.p[2] = 150;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 7);

    // a.p[0] = 125;
    // a.p[1] = -105;// -9.75
    // a.p[2] = 70;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 5);
    // // 125.0, -105.0
    // // 335.0, 105.0

    // a.p[0] = 125;//335;
    // a.p[1] = 105;// -9.75
    // a.p[2] = 70;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 5);
    // actions.emplace_back(16, dummy, 0, 2);

    // a.p[0] = 350;
    // a.p[1] = -120;// -9.75
    // a.p[2] = 50;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 7);

    // a.j[0] = 0 * deg2rad;
    // a.j[1] = 0 * deg2rad;
    // a.j[2] = 0 * deg2rad;
    // a.j[3] = 0 * deg2rad;
    // actions.emplace_back(4, a, 0, 10);

    // a.j[0] = -20 * deg2rad;
    // a.j[1] = 30 * deg2rad;
    // a.j[2] = 120 * deg2rad;
    // a.j[3] = 23 * deg2rad;
    // actions.emplace_back(4, a, 0, 10);

    // a.j[0] = -20 * deg2rad;
    // a.j[1] = 32 * deg2rad;
    // a.j[2] = 125 * deg2rad;
    // a.j[3] = 23 * deg2rad;
    // actions.emplace_back(4, a, 0, 1);

    // actions.emplace_back(16, dummy, 0, 1);
    // actions.emplace_back(2, dummy, 0, 1);

    // actions.emplace_back(4, a, 0, 1);
    // a.j[0] = -20 * deg2rad;
    // a.j[1] = 30 * deg2rad;
    // a.j[2] = 120 * deg2rad;
    // a.j[3] = 23 * deg2rad;

    // a.p[0] = 200;
    // a.p[1] = 0;// -9.75
    // a.p[2] = 100;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = 15 * deg2rad;
    // a.j[1] = 45 * deg2rad;
    // a.j[2] = 89 * deg2rad;
    // a.j[3] = 39 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = 15 * deg2rad;
    // a.j[1] = 47 * deg2rad;
    // a.j[2] = 93 * deg2rad;
    // a.j[3] = 39 * deg2rad;
    // actions.emplace_back(4, a, 0, 1);
    // actions.emplace_back(16, dummy, 0, 1);

    // a.j[0] = 15 * deg2rad;
    // a.j[1] = 45 * deg2rad;
    // a.j[2] = 89 * deg2rad;
    // a.j[3] = 39 * deg2rad;
    // actions.emplace_back(4, a, 0, 1);

    // a.p[0] = 200;
    // a.p[1] = 0;// -9.75
    // a.p[2] = 100;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = 27 * deg2rad;
    // a.j[1] = 29 * deg2rad;
    // a.j[2] = 124 * deg2rad;
    // a.j[3] = 24 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = 19 * deg2rad;
    // a.j[1] = 54 * deg2rad;
    // a.j[2] = 78 * deg2rad;
    // a.j[3] = 47 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);
    // actions.emplace_back(16, dummy, 0, 1);

    // a.j[0] = 12 * deg2rad;
    // a.j[1] = 57 * deg2rad;
    // a.j[2] = 71 * deg2rad;
    // a.j[3] = 48 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = 27 * deg2rad;
    // a.j[1] = 30 * deg2rad;
    // a.j[2] = 90 * deg2rad;
    // a.j[3] = 24 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // ovo
    // a.j[0] = 27 * deg2rad;
    // a.j[1] = 20 * deg2rad;
    // a.j[2] = 100 * deg2rad;
    // a.j[3] = 90 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // a.p[0] = 200;
    // a.p[1] = 0;// -9.75
    // a.p[2] = 100;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = -20 * deg2rad;
    // a.j[1] = 30 * deg2rad;
    // a.j[2] = 120 * deg2rad;
    // a.j[3] = 23 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // a.j[0] = -20 * deg2rad;
    // a.j[1] = 32 * deg2rad;
    // a.j[2] = 125 * deg2rad;
    // a.j[3] = 23 * deg2rad;
    // actions.emplace_back(4, a, 0, 1);

    actions.emplace_back(16, dummy, 0, 1);
    // actions.emplace_back(1, dummy, 0, 1);
    // ovo
    // a.j[0] = -20 * deg2rad;
    // a.j[1] = 30 * deg2rad;
    // a.j[2] = 120 * deg2rad;
    // a.j[3] = 23 * deg2rad;
    // actions.emplace_back(4, a, 0, 5);

    // a.p[0] = 200;
    // a.p[1] = 0;// -9.75
    // a.p[2] = 100;
    // a.p[3] = PI;
    // a = InvKine(a);
    // actions.emplace_back(4, a, 0, 5);
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
    ros::Subscriber action_msg_sub = n.subscribe("/checkers/robot_action_msg", 50, robot_point_msg_callback);
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
    has_actions = 0;
    current_trajectory.Finish();

    robotState requested, req_inter, req_inter2, dummy;

    requested.p[0] = 200;
    requested.p[1] = 100;// -9.75
    requested.p[2] = 100;
    requested.p[3] = PI;

    req_inter.p[0] = 120+250;
    req_inter.p[1] = -60;
    req_inter.p[2] = 100;
    req_inter.p[3] = PI;

    req_inter2.p[0] = 360;
    req_inter2.p[1] = 0;
    req_inter2.p[2] = 66;
    req_inter2.p[3] = PI;

    robotState zp;
    zp.j[0] = 2*deg2rad;
    zp.j[1] = 90*deg2rad;
    zp.j[2] = 2*deg2rad;
    zp.j[3] = -2*deg2rad;
    printf("Opa Gangam Style\n");
    auto fl_d(ForKine(zp/*InvKine(requested)*/));
    for(int i = 0; i < 6; i++) {
        printf("%.3lf\n", (i<3)?fl_d.p[i]:(fl_d.p[i]*rad2deg));
    }
    printf("Op op op Opa\n");

    return 0;
    requested = InvKine(requested, 0);
    for(int i = 0; i < 4; i++) requested.j[i] = 0;
    //     requested.j[0] = PI/2;
    req_inter = InvKine(req_inter, 0);
    req_inter2 = InvKine(req_inter2, 0);
    std::cout << "Uglovi:" << requested.j[0] << requested.j[1] << requested.j[2] << requested.j[3] << std::endl;

    printf("Trn:\t");
    for(int i = 0; i < 4; i++)
        printf("%lf\t", robot_current.j[i]);
    printf("\n");

    AddActions();
    // actions.emplace_back(1, dummy, 0, 4);
  //  actions.emplace_back(4, requested, 0, 7);
    // actions.emplace_back(4, requested, 0, 7);

//     actions.emplace_back(2, dummy, 0, 4);
//     actions.emplace_back(16, dummy, 0, 2);

//     actions.emplace_back(4, req_inter, 0, 5);
//    // actions.emplace_back(16, dummy, 0, 5);
//    // actions.emplace_back(1, dummy, 0, 4);
//     actions.emplace_back(4, req_inter2, 0, 7);


    // Trajectory6 z(robot_current, requested, req_inter, 7);
    current_trajectory = Trajectory(robot_current, robot_current, 5);
    current_trajectory.Finish();
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
        pub_vel.position.resize(4);
        pub_vel.name.resize(4);
        if(current_trajectory.IsFinished()) NextTrajectory();
        auto q = current_trajectory.GetPos(robot_current);
        for(int i = 0; i < 4; i++) pub_vel.position[i] = q[i];
        cpr_pos_pub.publish(pub_vel);


        loop_rate.sleep();
    }

    return 0;
}
