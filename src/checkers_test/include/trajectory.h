#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <iterator>
#include "moving.h"

using std::vector;
namespace ch {

class Trajectory {
public:
    // begin - a, end - b, inter - v, duration in seconds - tf
    Trajectory(robotState a, robotState b, robotState v,
               double tf) {
        a = InvKine(a, 0);
        b = InvKine(b, 0);
        v = InvKine(v, 0);
        coef.resize(4, vector<double>(7));
        double th_v, th_s, th_f;
        for(int i = 0; i < 4; i++) {
            th_s = a.j[i];
            th_f = b.j[i];
            th_v = v.j[i];
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
    vector<double> GetVel(const robotState &robot_current, int tick=-1) {
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
        double alpha = 100;
        for(int i = 0; i < 4; i++, pt = 1) {
            for(int j = 0; j < 7; j++, pt *= t)
                ret[i] += coef[i][j] * pt;  // get trajectory angle
            // make angle to velocity
            ret[i] = alpha * (ret[i] - robot_current.j[i]) / (1./update_f);
        }
        return ret;
    }
    bool IsFinished() const {
        return finished;
    }
private:
    vector<vector<double> > coef;
    int current_iter;
    double duration;
    bool finished;
};

}
#endif // TRAJECTORY_H
