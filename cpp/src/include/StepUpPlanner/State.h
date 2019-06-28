#ifndef STEPUPPLANNER_STATE_H
#define STEPUPPLANNER_STATE_H

#include <casadi/casadi.hpp>

namespace StepUpPlanner {
    class State;
}

class StepUpPlanner::State {

    casadi::MX m_position;
    casadi::MX m_velocity;

    casadi::MX m_stacked;


public:

    State();

    State(const State& other);

    State(State&& other);

    ~State();

    void operator=(const State& other);

    void setPosition(double px, double py, double pz);

    void setVelocity(double vx, double vy, double vz);

    casadi::MX& getPosition();

    casadi::MX& getVelocity();

    casadi::MX& getState();
};

#endif // STEPUPPLANNER_STATE_H
