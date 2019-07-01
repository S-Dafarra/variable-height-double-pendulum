#include <StepUpPlanner/Phase.h>
#include <iostream>

StepUpPlanner::Phase::Phase(PhaseType phase)
    : m_minDuration(0.5)
      , m_maxDuration(2.0)
      , m_desiredDuration(1.0)
      , m_phase(phase)
{ }

StepUpPlanner::Phase::Phase(const Step *left, const Step *right)
{
    if (left && right) {
        m_phase = StepUpPlanner::PhaseType::DOUBLE_SUPPORT;
        m_steps.left = *left;
        m_steps.right = *right;
    } else if (left) {
        m_phase = StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT;
        m_steps.left = *left;
    } else if (right) {
        m_phase = StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT;
        m_steps.right = *right;
    } else {
        m_phase = StepUpPlanner::PhaseType::FLYING;
    }

}

StepUpPlanner::Phase::~Phase()
{

}

bool StepUpPlanner::Phase::setDurationSettings(double minimumDuration, double maximumDuration, double desiredDuration)
{
    if (minimumDuration > maximumDuration) {
        std::cerr << "[ERROR][StepUpPlanner::Phase::setDurationSettings] The minimumDuration is greater than the maximumDuration." << std::endl;
        return false;
    }

    if ((desiredDuration < minimumDuration) || (desiredDuration > maximumDuration)) {
        std::cerr << "[ERROR][StepUpPlanner::Phase::setDurationSettings] The desiredDuration is not in the interval [minimumDuration, maximumDuration]." << std::endl;
        return false;
    }

    m_minDuration = minimumDuration;
    m_maxDuration = maximumDuration;
    m_desiredDuration = desiredDuration;

    return true;
}

void StepUpPlanner::Phase::setDesiredLeftPosition(double px, double py, double pz)
{
    m_steps.left.setPosition(px, py, pz);
}

void StepUpPlanner::Phase::setDesiredRightPosition(double px, double py, double pz)
{
    m_steps.right.setPosition(px, py, pz);
}

StepUpPlanner::PhaseType StepUpPlanner::Phase::getPhase() const
{
    return m_phase;
}

casadi::MX &StepUpPlanner::Phase::leftPosition()
{
    return m_steps.left.position();
}

casadi::MX &StepUpPlanner::Phase::rightPosition()
{
    return m_steps.right.position();
}

const StepUpPlanner::Step &StepUpPlanner::Phase::getLeftStep() const
{
    return m_steps.left;
}

const StepUpPlanner::Step &StepUpPlanner::Phase::getRightStep() const
{
    return m_steps.right;
}

casadi::MX &StepUpPlanner::Phase::duration()
{
    return m_duration;
}
