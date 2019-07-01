#include <StepUpPlanner/Control.h>

StepUpPlanner::FootControl::FootControl()
    : m_multiplier(1)
      , m_CoP(2)
{ }

StepUpPlanner::FootControl::FootControl(const StepUpPlanner::FootControl &other)
{
    operator=(other);
}

StepUpPlanner::FootControl::FootControl(StepUpPlanner::FootControl &&other)
{
    operator=(other);
}

StepUpPlanner::FootControl::~FootControl()
{ }

void StepUpPlanner::FootControl::operator=(const StepUpPlanner::FootControl &other)
{
    m_multiplier = other.m_multiplier;
    m_CoP = other.m_CoP;
}

casadi::MX &StepUpPlanner::FootControl::cop()
{
    return m_CoP;
}

casadi::MX &StepUpPlanner::FootControl::multiplier()
{
    return m_multiplier;
}

StepUpPlanner::Control::Control()
{
    m_stacked = casadi::MX::vertcat({m_controls.left.multiplier(),
                                     m_controls.left.cop(),
                                     m_controls.right.multiplier(),
                                     m_controls.right.cop()});
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
    m_controls = other.m_controls;

    m_stacked = casadi::MX::vertcat({m_controls.left.multiplier(),
                                     m_controls.left.cop(),
                                     m_controls.right.multiplier(),
                                     m_controls.right.cop()});}

casadi::MX &StepUpPlanner::Control::controlVector()
{
    return m_stacked;
}

StepUpPlanner::SideDependentObject<StepUpPlanner::FootControl> &StepUpPlanner::Control::controls()
{
    return m_controls;
}
