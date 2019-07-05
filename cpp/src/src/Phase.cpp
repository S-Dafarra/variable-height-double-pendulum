#include <StepUpPlanner/Phase.h>
#include <iostream>

StepUpPlanner::Phase::Phase()
    : m_duration(1,1)
      ,m_minDuration(0.5)
      , m_maxDuration(2.0)
      , m_desiredDuration(1.0)
      , m_phase(StepUpPlanner::PhaseType::UNDEFINED)
{ }

StepUpPlanner::Phase::Phase(PhaseType phase)
    : m_duration(1,1)
      ,m_minDuration(0.5)
      , m_maxDuration(2.0)
      , m_desiredDuration(1.0)
      , m_phase(phase)
{ }

StepUpPlanner::Phase::Phase(const Step *left, const Step *right)
    : m_duration(1,1)
      ,m_minDuration(0.5)
      , m_maxDuration(2.0)
      , m_desiredDuration(1.0)
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

void StepUpPlanner::Phase::setLeftPosition(double px, double py, double pz)
{
    m_steps.left.setPosition(px, py, pz);
}

void StepUpPlanner::Phase::setRightPosition(double px, double py, double pz)
{
    m_steps.right.setPosition(px, py, pz);
}

StepUpPlanner::PhaseType StepUpPlanner::Phase::getPhaseType() const
{
    return m_phase;
}

const casadi::DM &StepUpPlanner::Phase::duration() const
{
    return m_duration;
}

casadi::DM &StepUpPlanner::Phase::duration()
{
    return m_duration;
}

double StepUpPlanner::Phase::minDuration() const
{
    return m_minDuration;
}

double StepUpPlanner::Phase::maxDuration() const
{
    return m_maxDuration;
}

double StepUpPlanner::Phase::desiredDuration() const
{
    return m_desiredDuration;
}

casadi::DM &StepUpPlanner::Phase::leftPosition()
{
    return m_steps.left.position();
}

const casadi::DM &StepUpPlanner::Phase::leftPosition() const
{
    return m_steps.left.position();
}

casadi::DM &StepUpPlanner::Phase::rightPosition()
{
    return m_steps.right.position();
}

const casadi::DM &StepUpPlanner::Phase::rightPosition() const
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

const std::vector<StepUpPlanner::State> &StepUpPlanner::Phase::states() const
{
    return m_statesSolution;
}

std::vector<StepUpPlanner::State> &StepUpPlanner::Phase::states()
{
    return m_statesSolution;
}

const std::vector<StepUpPlanner::Control> &StepUpPlanner::Phase::controls() const
{
    return m_controlsSolution;
}

std::vector<StepUpPlanner::Control> &StepUpPlanner::Phase::controls()
{
    return m_controlsSolution;
}

