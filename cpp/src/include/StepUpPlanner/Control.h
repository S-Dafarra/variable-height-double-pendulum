#ifndef STEPUPPLANNER_CONTROL_H
#define STEPUPPLANNER_CONTROL_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/SideDependentObject.h>

namespace StepUpPlanner {
    class FootControl;
    class Control;
}

class StepUpPlanner::FootControl {

    casadi::DM m_multiplier;
    casadi::DM m_CoP;

public:

    FootControl();

    ~FootControl();

    //in foot coordinates
    casadi::DM &cop();

    const casadi::DM &cop() const;

    void setCoP(double copX, double copY);

    casadi::DM &multiplier();

    const casadi::DM &multiplier() const;

    void setMultiplier(double multiplier);

    void zero();
};

class StepUpPlanner::Control {

    StepUpPlanner::SideDependentObject<StepUpPlanner::FootControl> m_controls;
    casadi::DM m_acceleration;

public:

    Control();

    ~Control();

    const casadi::DM& acceleration() const;

    casadi::DM& acceleration();

    StepUpPlanner::FootControl &left();

    const StepUpPlanner::FootControl &left() const;

    StepUpPlanner::FootControl &right();

    const StepUpPlanner::FootControl &right() const;

    void zero();
};

#endif // STEPUPPLANNER_CONTROL_H
