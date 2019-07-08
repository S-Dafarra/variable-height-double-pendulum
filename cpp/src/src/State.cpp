#include <StepUpPlanner/State.h>
#include <cassert>

StepUpPlanner::State::State()
    : m_position(3, 1)
      , m_velocity(3, 1)
{ }

StepUpPlanner::State::~State()
{ }

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

casadi::DM &StepUpPlanner::State::position()
{
    return m_position;
}

double StepUpPlanner::State::position(size_t i) const
{
    assert(i < 3);
    return static_cast<double>(m_position(i));
}

const casadi::DM &StepUpPlanner::State::position() const
{
    return m_position;
}

casadi::DM &StepUpPlanner::State::velocity()
{
    return  m_velocity;
}

double StepUpPlanner::State::velocity(size_t i) const
{
    assert(i < 3);
    return static_cast<double>(m_velocity(i));
}

const casadi::DM &StepUpPlanner::State::velocity() const
{
    return  m_velocity;
}

void StepUpPlanner::State::zero()
{
    setPosition(0.0, 0.0, 0.0);
    setVelocity(0.0, 0.0, 0.0);
}
