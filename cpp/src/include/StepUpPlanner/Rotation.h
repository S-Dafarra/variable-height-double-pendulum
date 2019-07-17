#ifndef STEPUPPLANNER_ROTATION_H
#define STEPUPPLANNER_ROTATION_H

#include <casadi/casadi.hpp>

namespace StepUpPlanner {
    class Rotation;
}

class StepUpPlanner::Rotation {

    casadi::DM m_rotation;
    casadi::DM skew(double i, double j, double k);

public:

    Rotation();

    Rotation(const casadi::DM& matrix);

    void setFromQuaternion(double realPart, double i, double j, double k);

    const casadi::DM& asMatrix() const;

    static Rotation Identity();
};

#endif // STEPUPPLANNER_ROTATION_H
