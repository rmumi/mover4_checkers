#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <iterator>
#include "moving.h"

using std::vector;
namespace ch {

class Trajectory {
public:
  // horrible choice of ways to do multiple polynomials, but works
    Trajectory(robotState a, robotState b, double tf);
    Trajectory(robotState a, robotState b, robotState v, double tf);
    Trajectory(robotState a, robotState b, robotState v, double tf, double tv);
    Trajectory(robotState a, robotState b, double tf,  bool third);
    vector<double> GetPos(const robotState &robot_current, int tick=-1);
    bool IsFinished() const;
    void Finish();
private:
    Trajectory();
    vector<vector<double> > coef;
    int current_iter;
    int num_coef;
    double duration;  // time is normalised
    bool finished;
};

}
#endif // TRAJECTORY_H
