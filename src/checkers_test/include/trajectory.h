#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <iterator>
#include "moving.h"

using std::vector;
namespace ch {

class Trajectory6 {
public:
    Trajectory6(robotState a, robotState b, robotState v, double tf);
    vector<double> GetVel(const robotState &robot_current, int tick=-1);
    bool IsFinished() const;
    void Finish();
private:
    Trajectory6();
    vector<vector<double> > coef;
    int current_iter;
    double duration;
    bool finished;
};

class Trajectory5 {
public:
    Trajectory5(robotState a, robotState b, double tf);
    vector<double> GetVel(const robotState &robot_current, int tick=-1);
    bool IsFinished() const;
    void Finish();
private:
    Trajectory5();
    vector<vector<double> > coef;
    int current_iter;
    double duration;
    bool finished;
};

}
#endif // TRAJECTORY_H
