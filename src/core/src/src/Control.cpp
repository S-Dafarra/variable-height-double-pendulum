#include <StepUpPlanner/Control.h>
#include <cassert>

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

double StepUpPlanner::FootControl::cop(size_t i) const
{
    assert(i < 2);
    return static_cast<double>(m_CoP(i));
}

void StepUpPlanner::FootControl::setCoP(double copX, double copY)
{
    m_CoP(0) = copX;
    m_CoP(1) = copY;
}

casadi::DM &StepUpPlanner::FootControl::multiplier()
{
    return m_multiplier;
}

const casadi::DM &StepUpPlanner::FootControl::multiplier() const
{
    return m_multiplier;
}

void StepUpPlanner::FootControl::setMultiplier(double multiplier)
{
    m_multiplier = multiplier;
}

void StepUpPlanner::FootControl::zero()
{
    m_multiplier = 0.0;
    m_CoP(0) = 0.0;
    m_CoP(1) = 0.0;
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

double StepUpPlanner::Control::acceleration(size_t i) const
{
    assert(i < 3);
    return static_cast<double>(m_acceleration(i));
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

void StepUpPlanner::Control::zero()
{
    m_acceleration(0) = 0.0;
    m_acceleration(1) = 0.0;
    m_acceleration(2) = 0.0;
    m_controls.left.zero();
    m_controls.right.zero();
}
