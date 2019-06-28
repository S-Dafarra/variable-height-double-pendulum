#include <StepUpPlanner/State.h>

StepUpPlanner::State::State()
    : m_position(3)
      , m_velocity(3)
{
    m_stacked = casadi::MX::vertcat({m_position, m_velocity});
}

StepUpPlanner::State::State(const StepUpPlanner::State &other)
{
    this->operator=(other);
}

StepUpPlanner::State::State(StepUpPlanner::State &&other)
{
    this->operator=(other);
}

StepUpPlanner::State::~State()
{ }

void StepUpPlanner::State::operator=(const StepUpPlanner::State &other)
{
    m_position = other.m_position;
    m_velocity = other.m_velocity;
    m_stacked = casadi::MX::vertcat({m_position, m_velocity});
}

casadi::MX &StepUpPlanner::State::getState()
{
    return m_stacked;
}

void StepUpPlanner::State::setPosition(double px, double py, double pz)
{
    m_position(0) = px;
    m_position(1) = py;
    m_position(2) = pz;
}

void StepUpPlanner::State::setVelocity(double vx, double vy, double vz)
{
    m_velocity(0) = vx;
    m_velocity(1) = vy;
    m_velocity(2) = vz;
}

casadi::MX &StepUpPlanner::State::getPosition()
{
    return m_position;
}

casadi::MX &StepUpPlanner::State::getVelocity()
{
    return  m_velocity;
}
