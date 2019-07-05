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

    const casadi::DM& position() const;

    casadi::DM& velocity();

    const casadi::DM& velocity() const;

    void zero();
};

#endif // STEPUPPLANNER_STATE_H
