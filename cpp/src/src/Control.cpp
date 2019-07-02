#include <StepUpPlanner/Control.h>

StepUpPlanner::FootControl::FootControl()
    : m_multiplier(1)
      , m_CoP(2)
{ }

StepUpPlanner::FootControl::~FootControl()
{ }

void StepUpPlanner::FootControl::setCoP(double copX, double copY)
{
    m_CoP(0) = copX;
    m_CoP(1) = copY;
}

const casadi::DM &StepUpPlanner::FootControl::cop() const
{
    return m_CoP;
}

void StepUpPlanner::FootControl::setMultiplier(double u)
{
    m_multiplier(u);
}

const casadi::DM &StepUpPlanner::FootControl::multiplier() const
{
    return m_multiplier;
}

StepUpPlanner::Control::Control()
{ }


StepUpPlanner::Control::~Control()
{ }

StepUpPlanner::FootControl &StepUpPlanner::Control::leftControl()
{
    return m_controls.left;
}

const StepUpPlanner::FootControl &StepUpPlanner::Control::leftControl() const
{
    return m_controls.left;
}

StepUpPlanner::FootControl &StepUpPlanner::Control::rightControl()
{
    return m_controls.right;
}

const StepUpPlanner::FootControl &StepUpPlanner::Control::rightControl() const
{
    return m_controls.right;
}
