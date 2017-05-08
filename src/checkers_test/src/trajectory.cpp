#include "trajectory.h"

namespace ch {

// begin - a, end - b, inter - v, duration in seconds - tf
Trajectory6::Trajectory6(robotState a, robotState b, robotState v,
           double tf) {
    coef.resize(4, vector<double>(7));
    double th_v, th_s, th_f;
    for(int i = 0; i < 4; i++) {
        th_s = a.j[i];
        th_f = b.j[i];
        th_v = v.j[i];
        if(fabs(th_s - th_f) < 1e-3) th_s = th_f;
        coef[i][0] = th_s;
        coef[i][1] = 0;
        coef[i][2] = 0;
        coef[i][3] = 2*(32*(th_v-th_s)-11*(th_f-th_s))/(tf*tf*tf);
        coef[i][4] = -3*(64*(th_v-th_s)-27*(th_f-th_s))/(tf*tf*tf*tf);
        coef[i][5] = 3*(64*(th_v-th_s)-30*(th_f-th_s))/(tf*tf*tf*tf*tf);
        coef[i][6] = -32*(2*(th_v-th_s)-(th_f-th_s))/(tf*tf*tf*tf*tf*tf);
    }
    duration = tf;
    current_iter = 0;
    finished = 0;
}

vector<double> Trajectory6::GetVel(const robotState &robot_current, int tick) {
    if(tick == -1)
        tick = current_iter++;
    double t = 1./update_f * tick, pt = 1;
    vector<double> ret(4, 0.0);
    if(finished) return ret;
    if(t > duration) {
        t = duration;
        std::cout << "This shouldn't happen twice" << std::endl;
        finished = 1;
    }
    double alpha = 200;
    for(int i = 0; i < 4; i++, pt = 1) {
        ret[i] = 0;
        for(int j = 0; j < 7; j++, pt *= t)
            ret[i] += coef[i][j] * pt;  // get trajectory angle
        // make angle to velocity
        printf("Ugao #%d razlika: curr: %lf new: %lf diff: %lf%%\n", i, robot_current.j[i], ret[i], (ret[i] - robot_current.j[i])/robot_current.j[i] * 100);
        ret[i] = alpha * (ret[i] - robot_current.j[i]) / (1./update_f);

    }
    return ret;
}

bool Trajectory6::IsFinished() const {
    return finished;
}



// begin - a, end - b, duration in seconds - tf
Trajectory5::Trajectory5(robotState a, robotState b, double tf) {
    coef.resize(4, vector<double>(6));
    duration = 0;
    // -6 | 6 | -3 | -3 | -1/2 | 1/2
    // 15 | -15 | 8 | 7 | 3/2 | -1
    // -10 | 10 | -6 | -4 | -3/2 | 1/2
    // 0 | 0 | 0 | 0 | 1/2 | 0
    // 0 | 0 | 1 | 0 | 0 | 0
    // 1 | 0 | 0 | 0 | 0 | 0
    for(int i = 0; i < 4; i++) {  // time is normalised
        double th_s = a.j[i];
        double th_f = b.j[i];
        if(fabs(th_s - th_f) < 1e-3) th_s = th_f;
        coef[i][0] = th_s;
        coef[i][1] = 0;
        coef[i][2] = 0;
        coef[i][3] = -10*th_s+10*th_f;
        coef[i][4] = 15*th_s-15*th_f;
        coef[i][5] = -6*th_s+6*th_f;
        printf("Zglob %d:\t", i);
        for(int j = 0; j < 6; j++)
            printf("%lf\t", coef[i][j]);
        printf("\n");
    }
    duration = tf;
    current_iter = 0;
    finished = 0;
}

vector<double> Trajectory5::GetVel(const robotState &robot_current, int tick) {
    if(tick == -1)
        tick = current_iter++;
    double t = (1./update_f * tick)/duration, pt = 1;  // time is normalised
    vector<double> ret(4, 0.0);
    if(finished) return ret;
    if(t > 1) {
        t = 1;
        std::cout << "This shouldn't happen twice" << std::endl;
        finished = 1;
    }
    double alpha = 200;
    for(int i = 0; i < 4; i++, pt = 1) {
        ret[i] = 0;
        for(int j = 0; j < 6; j++, pt *= t)
            ret[i] += coef[i][j] * pt;  // get trajectory angle
        // make angle to velocity
        if(tick == 0)
        printf("Ugao #%d razlika: curr: %lf new: %lf diff: %lf%%\n", i, robot_current.j[i], ret[i], (ret[i] - robot_current.j[i])/robot_current.j[i] * 100);
        ret[i] = alpha * (ret[i] - robot_current.j[i]) / (1./update_f);

    }
    return ret;
}

bool Trajectory5::IsFinished() const {
    return finished;
}




}
