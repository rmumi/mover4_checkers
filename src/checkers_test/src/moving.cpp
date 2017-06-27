#include "moving.h"
#include "trajectory.h"

robotState robot_current;

deque<RobotAction> actions;
Trajectory current_trajectory(robot_current, robot_current, 1);
ros::Publisher cpr_commands_pub, robot_state_msg_pub, robot_sig_pub;
bool has_actions;

void joint_states_callback(const sensor_msgs::JointState &msg) {
     for(int i = 0; i < 4; i++)
         robot_current.j[i] = msg.position[i];
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
            actions.front()._q, static_cast<double>(actions.front().wait)/update_f);
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
                                        static_cast<double>(fin.wait+mid.wait)/update_f,
                                        static_cast<double>(mid.wait)/update_f);
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
    // for(int i = 0; i < 4; i++) {
    //     for(int j = 0; j < 4; j++) {
    //         printf("%.2lf\t", A[i][j]);
    //     }
    //     printf("\n");
    // }
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

    double d = sqrt(11*11 + 15*15);
    double th_s = atan2(11, 15);
    xr = xr + d * cos(th_s + th0);
    yr = yr + d * sin(th_s + th0);
    th0 = atan2(yr, xr);


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

int main(int argc, char** argv) {

    ros::init(argc, argv, "checkers_trajectory_planning");

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

    printf("Trn:\t");
    for(int i = 0; i < 4; i++)
        printf("%lf\t", robot_current.j[i]);
    printf("\n");

    current_trajectory = Trajectory(robot_current, robot_current, 5);
    current_trajectory.Finish();

    // for(int i = 0; i < 4; i++) {
    //   requested.p[0] = 200;
    //   requested.p[1] = 300;
    //   requested.p[2] = 100;
    //   requested.p[3] = PI;
    //   requested = InvKine(requested, i);
    //   printf("Req:\t");
    //   for(int i = 0; i < 4; i++)
    //       printf("%lf\t", requested.j[i]*rad2deg);
    //   printf("\n");
    // }

    robotState to_work_with;
    std_msgs::Float64MultiArray info;

    while(ros::ok()) {

        ros::spinOnce();  // process callbacks, so it's not late

        // publish the current state and position
        info.data.resize(4 + 6);
        for(int i = 0; i < 4; i++) info.data[i] = robot_current.j[i] * rad2deg;
        to_work_with = ForKine(robot_current);
        for(int i = 0; i < 6; i++) info.data[i+4] = to_work_with.p[i];
        robot_state_msg_pub.publish(info);

        sensor_msgs::JointState pub_pos;
        pub_pos.header.stamp = ros::Time::now();
        pub_pos.position.resize(4);
        pub_pos.name.resize(4);
        if(current_trajectory.IsFinished()) NextTrajectory();
        auto q = current_trajectory.GetPos(robot_current);
        for(int i = 0; i < 4; i++) pub_pos.position[i] = q[i];
        cpr_pos_pub.publish(pub_pos);
        // std::cout << "Trenut:\t" << robot_current.j[0] * rad2deg << "\t" << robot_current.j[1] * rad2deg << "\t" << robot_current.j[2] * rad2deg<< "\t" << robot_current.j[3] * rad2deg<< std::endl;
        // std::cout << "Zahtje:\t" << pub_pos.position[0] * rad2deg << "\t" << pub_pos.position[0] * rad2deg << "\t" << pub_pos.position[0] * rad2deg<< "\t" << pub_pos.position[0] * rad2deg<< std::endl;
        // std::cout << "Greska:\t" << (pub_pos.position[0] - robot_current.j[0]) * rad2deg << "\t" << (pub_pos.position[1] - robot_current.j[1]) * rad2deg << "\t" << (pub_pos.position[2] - robot_current.j[2]) * rad2deg<< "\t" << (pub_pos.position[3] - robot_current.j[3]) * rad2deg<< std::endl;

        loop_rate.sleep();
    }

    return 0;
}
