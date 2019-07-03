#include <StepUpPlanner/Control.h>

StepUpPlanner::FootControl::FootControl()
    : m_multiplier(1,1)
      , m_CoP(2,1)
{ }

StepUpPlanner::FootControl::~FootControl()
{ }

casadi::DM &StepUpPlanner::FootControl::cop()
{
    return m_CoP;
}

const casadi::DM &StepUpPlanner::FootControl::cop() const
{
    return m_CoP;
}

casadi::DM &StepUpPlanner::FootControl::multiplier()
{
    return m_multiplier;
}

const casadi::DM &StepUpPlanner::FootControl::multiplier() const
{
    return m_multiplier;
}

StepUpPlanner::Control::Control()
    : m_acceleration(3,1)
{ }


StepUpPlanner::Control::~Control()
{ }

const casadi::DM &StepUpPlanner::Control::acceleration() const
{
    return m_acceleration;
}

casadi::DM &StepUpPlanner::Control::acceleration()
{
    return m_acceleration;
}

StepUpPlanner::FootControl &StepUpPlanner::Control::left()
{
    return m_controls.left;
}

const StepUpPlanner::FootControl &StepUpPlanner::Control::left() const
{
    return m_controls.left;
}

StepUpPlanner::FootControl &StepUpPlanner::Control::right()
{
    return m_controls.right;
}

const StepUpPlanner::FootControl &StepUpPlanner::Control::right() const
{
    return m_controls.right;
}
