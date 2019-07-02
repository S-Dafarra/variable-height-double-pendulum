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
    void setCoP(double copX, double copY);

    const casadi::DM &cop() const;

    void setMultiplier(double u);

    const casadi::DM &multiplier() const;
};

class StepUpPlanner::Control {

    StepUpPlanner::SideDependentObject<StepUpPlanner::FootControl> m_controls;

public:

    Control();

    ~Control();

    StepUpPlanner::FootControl &left();

    const StepUpPlanner::FootControl &left() const;

    StepUpPlanner::FootControl &right();

    const StepUpPlanner::FootControl &right() const;


};

#endif // STEPUPPLANNER_CONTROL_H
