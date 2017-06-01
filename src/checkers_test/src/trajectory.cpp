#include "trajectory.h"

namespace ch {

// begin - a, end - b, inter - v, duration in seconds - tf
Trajectory::Trajectory(robotState a, robotState b, robotState v,
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
        coef[i][3] = 2*(32*(th_v-th_s)-11*(th_f-th_s));
        coef[i][4] = -3*(64*(th_v-th_s)-27*(th_f-th_s));
        coef[i][5] = 3*(64*(th_v-th_s)-30*(th_f-th_s));
        coef[i][6] = -32*(2*(th_v-th_s)-(th_f-th_s));
        // printf("Zglob %d:\t", i);
        // for(int j = 0; j < 7; j++)
        //     printf("%lf\t", coef[i][j]);
        // printf("\n");
    }
    duration = tf;
    current_iter = 0;
    finished = 0;
    num_coef = 7;
}

Trajectory::Trajectory(robotState a, robotState b, robotState v,
           double tf, double tv) {
    coef.resize(4, vector<double>(7));
    double th_v, th_s, th_f;
    tv = tv / tf;
    // std::cout<<"tv:" << tv<<"\n";
    for(int i = 0; i < 4; i++) {
        th_s = a.j[i];
        th_f = b.j[i];
        th_v = v.j[i];
        if(fabs(th_s - th_f) < 1e-3) th_s = th_f;
        coef[i][0] = th_s;
        coef[i][1] = 0;
        coef[i][2] = 0;
        coef[i][3] = 1/std::pow(1-tv, 3) * ((th_v-th_s)/std::pow(tv, 3)-(th_f-th_s)*(15-24*tv+10*tv*tv)*tv);
        coef[i][4] = -3/std::pow(1-tv, 3) * ((th_v-th_s)/std::pow(tv, 3)-(th_f-th_s)*(5-9*tv*tv+5*std::pow(tv, 3)));
        coef[i][5] = 3/std::pow(1-tv, 3) * ((th_v-th_s)/std::pow(tv, 3)-(th_f-th_s)*(8-9*tv+2*std::pow(tv, 3)));
        coef[i][6] = -1/std::pow(1-tv, 3) * ((th_v-th_s)/std::pow(tv, 3)-(th_f-th_s)*(10-15*tv+6*tv*tv));
        // printf("Zglob %d:\t", i);
        // for(int j = 0; j < 7; j++)
        //     printf("%lf\t", coef[i][j]);
        // printf("\n");
    }
    duration = tf;
    current_iter = 0;
    finished = 0;
    num_coef = 7;
}

// begin - a, end - b, duration in seconds - tf
Trajectory::Trajectory(robotState a, robotState b, double tf) {
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
        if(fabs(th_s - th_f) < 1e-4) th_s = th_f;
        coef[i][0] = th_s;
        coef[i][1] = 0;
        coef[i][2] = 0;
        coef[i][3] = -10*th_s+10*th_f;
        coef[i][4] = 15*th_s-15*th_f;
        coef[i][5] = -6*th_s+6*th_f;
        // printf("Zglob %d:\t", i);
        // for(int j = 0; j < 6; j++)
        //     printf("%lf\t", coef[i][j]);
        // printf("\n");
    }
    duration = tf;
    current_iter = 0;
    finished = 0;
    num_coef = 6;
}

vector<double> Trajectory::GetVel(const robotState &robot_current, int tick) {
    if(tick == -1)
        tick = current_iter++;
    double t = (1./update_f * tick) / duration, pt = 1;
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
        for(int j = 0; j < num_coef; j++, pt *= t)
            ret[i] += coef[i][j] * pt;  // get trajectory angle
        // make angle to velocity
        printf("Ugao #%d razlika: curr: %lf new: %lf diff: %lf%%\n", i, robot_current.j[i], ret[i], (ret[i] - robot_current.j[i]) * 100);
        ret[i] = alpha * (ret[i] - robot_current.j[i]) / (1./update_f);

    }
    return ret;
}

vector<double> Trajectory::GetPos(const robotState &robot_current, int tick) {
    if(tick == -1)
        tick = current_iter++;
    double t = (1./update_f * tick)/duration, pt = 1;  // time is normalised
    vector<double> ret(4, 0.0);
    for(int i = 0; i < 4; i++) ret[i] = robot_current.j[i];
    if(finished) { t = 1;
        //return ret;
    }
    if(t > 1) {
        t = 1;
        std::cout << "This shouldn't happen twice" << std::endl;
        finished = 1;
    }
    for(int i = 0; i < 4; i++, pt = 1) {
        ret[i] = 0;
        for(int j = 0; j < num_coef; j++, pt *= t)
            ret[i] += coef[i][j] * pt;  // get trajectory angle
        if(t >= 0.999 && (1./update_f * tick)/duration < 1.2) {
            printf("Ugao #%d razlika: curr: %lf new: %lf diff: %lf%%\n", i, robot_current.j[i] * rad2deg, ret[i] * rad2deg, (ret[i] - robot_current.j[i])*rad2deg);
        }
    }
    return ret;
}

bool Trajectory::IsFinished() const {
    return finished;
}

void Trajectory::Finish() {
    finished = 1;
}

}
