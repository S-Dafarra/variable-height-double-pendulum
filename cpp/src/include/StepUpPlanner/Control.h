#ifndef STEPUPPLANNER_CONTROL_H
#define STEPUPPLANNER_CONTROL_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/SideDependentObject.h>

namespace StepUpPlanner {
    class FootControl;
    class Control;
}

class StepUpPlanner::FootControl {
    casadi::MX m_multiplier;
    casadi::MX m_CoP;

public:

    FootControl();

    FootControl(const FootControl& other);

    FootControl(FootControl&& other);

    ~FootControl();

    void operator=(const FootControl& other);

    casadi::MX& cop();

    casadi::MX cop() const;

    casadi::MX& multiplier();

    casadi::MX multiplier() const;
};

class StepUpPlanner::Control {

    StepUpPlanner::SideDependentObject<StepUpPlanner::FootControl> m_controls;

    casadi::MX m_stacked;

public:

    Control();

    Control(const Control& other);

    Control(Control&& other);

    ~Control();

    void operator=(const Control& other);

    casadi::MX &controlVector();

    StepUpPlanner::FootControl &leftControl();

    StepUpPlanner::FootControl &rightControl();

};

#endif // STEPUPPLANNER_CONTROL_H
