#include <StepUpPlanner/Control.h>

StepUpPlanner::Control::Control()
    : m_leftMultiplier(1)
      , m_leftCoP(2)
      , m_rightMultiplier(1)
      , m_rightCoP(2)
{
    m_stacked = casadi::MX::vertcat({m_leftMultiplier, m_leftCoP, m_rightMultiplier, m_rightCoP});
}

StepUpPlanner::Control::Control(const StepUpPlanner::Control &other)
{
    this->operator=(other);
}

StepUpPlanner::Control::Control(StepUpPlanner::Control &&other)
{
    this->operator=(other);
}

StepUpPlanner::Control::~Control()
{ }

void StepUpPlanner::Control::operator=(const StepUpPlanner::Control &other)
{
    m_leftMultiplier = other.m_leftMultiplier;
    m_leftCoP = other.m_leftCoP;
    m_rightMultiplier = other.m_rightMultiplier;
    m_rightCoP = other.m_rightCoP;

    m_stacked = casadi::MX::vertcat({m_leftMultiplier, m_leftCoP, m_rightMultiplier, m_rightCoP});
}

casadi::MX &StepUpPlanner::Control::getControl()
{
    return m_stacked;
}

casadi::MX &StepUpPlanner::Control::getLeftMultiplier()
{
    return m_leftMultiplier;
}

casadi::MX &StepUpPlanner::Control::getLeftCoP()
{
    return m_leftCoP;
}

casadi::MX &StepUpPlanner::Control::getRightMultiplier()
{
    return m_rightMultiplier;
}

casadi::MX &StepUpPlanner::Control::getRightCoP()
{
    return m_rightCoP;
}
