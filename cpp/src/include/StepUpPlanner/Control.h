#ifndef STEPUPPLANNER_CONTROL_H
#define STEPUPPLANNER_CONTROL_H

#include <casadi/casadi.hpp>

namespace StepUpPlanner {
    class Control;
}

class StepUpPlanner::Control {

    casadi::MX m_leftMultiplier;
    casadi::MX m_leftCoP;
    casadi::MX m_rightMultiplier;
    casadi::MX m_rightCoP;

    casadi::MX m_stacked;

public:

    Control();

    Control(const Control& other);

    Control(Control&& other);

    ~Control();

    void operator=(const Control& other);

    casadi::MX &getControl();

    casadi::MX &getLeftMultiplier();

    casadi::MX &getLeftCoP();

    casadi::MX &getRightMultiplier();

    casadi::MX &getRightCoP();

};

#endif // STEPUPPLANNER_CONTROL_H
