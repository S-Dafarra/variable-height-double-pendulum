#ifndef STEPUPPLANNER_ROTATION_H
#define STEPUPPLANNER_ROTATION_H

#include <casadi/casadi.hpp>

namespace StepUpPlanner {
    class Rotation;
}

class StepUpPlanner::Rotation {

    casadi::DM m_rotation, m_quaternion;
    casadi::DM skew(double i, double j, double k);

    void computeQuaternion();

public:

    Rotation();

    Rotation(const casadi::DM& matrix);

    void setFromQuaternion(double realPart, double i, double j, double k);

    const casadi::DM& asMatrix() const;

    const casadi::DM& asQuaternion() const;

    double asQuaternion(size_t i) const;

    static Rotation Identity();
};

#endif // STEPUPPLANNER_ROTATION_H
