#include <StepUpPlanner/Phase.h>
#include <iostream>

StepUpPlanner::Phase::Phase(PhaseType phase)
    : m_minDuration(0.5)
      , m_maxDuration(2.0)
      , m_desiredDuration(1.0)
      , m_leftPosition(3)
      , m_rightPosition(3)
      , m_phase(phase)
{ }

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
    m_leftPosition(0) = px;
    m_leftPosition(1) = py;
    m_leftPosition(2) = pz;
}

void StepUpPlanner::Phase::setDesiredRightPosition(double px, double py, double pz)
{
    m_rightPosition(0) = px;
    m_rightPosition(1) = py;
    m_rightPosition(2) = pz;
}

StepUpPlanner::PhaseType StepUpPlanner::Phase::getPhase() const
{
    return m_phase;
}

casadi::MX &StepUpPlanner::Phase::getLeftPosition()
{
    return m_leftPosition;
}

casadi::MX &StepUpPlanner::Phase::getRightPosition()
{
    return m_rightPosition;
}

casadi::MX &StepUpPlanner::Phase::getDuration()
{
    return m_duration;
}
