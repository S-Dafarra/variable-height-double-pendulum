#ifndef STEPUPPLANNER_STATE_H
#define STEPUPPLANNER_STATE_H

#include <casadi/casadi.hpp>

namespace StepUpPlanner {
    class State;
}

class StepUpPlanner::State {

    casadi::DM m_position;
    casadi::DM m_velocity;

public:

    State();

    ~State();

    void setPosition(double px, double py, double pz);

    void setVelocity(double vx, double vy, double vz);

    casadi::DM& position();

    double position(size_t i) const;

    const casadi::DM& position() const;

    casadi::DM& velocity();

    double velocity(size_t i) const;

    const casadi::DM& velocity() const;

    void zero();
};

#endif // STEPUPPLANNER_STATE_H
