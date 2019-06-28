#include <StepUpPlanner/References.h>
#include <iostream>


StepUpPlanner::References::References()
    : m_finalStateAnticipation(0.3)
      , m_desiredLegLength(1.2)
{ }

StepUpPlanner::References::References(const StepUpPlanner::References &other)
{
    this->operator=(other);
}

StepUpPlanner::References::References(StepUpPlanner::References &&other)
{
    this->operator=(other);
}

StepUpPlanner::References::~References()
{ }

void StepUpPlanner::References::operator=(const StepUpPlanner::References &other)
{
    m_desiredState = other.m_desiredState;
    m_desiredControl = other.m_desiredControl;
    m_finalStateAnticipation = other.m_finalStateAnticipation;
    m_desiredLegLength = other.m_desiredLegLength;
}

void StepUpPlanner::References::setDesiredState(const StepUpPlanner::State &state)
{
    m_desiredState = state;
}

void StepUpPlanner::References::setDesiredControl(const StepUpPlanner::Control &control)
{
    m_desiredControl = control;
}

bool StepUpPlanner::References::setFinalStateAnticipation(double finalStateAnticipation)
{
    if ((finalStateAnticipation > 1.0) || (finalStateAnticipation < 0.0))
    {
        std::cerr << "[ERROR][StepUpPlanner::References::setFinalStateAnticipation] The final state anticipation is supposed to be in the interval [0.0, 1.0]." << std::endl;
        return false;
    }

    m_finalStateAnticipation = finalStateAnticipation;

    return true;
}

bool StepUpPlanner::References::setDesiredLegLength(double desiredLegLength)
{
    if (desiredLegLength < 0.0) {
        std::cerr << "[ERROR][StepUpPlanner::References::setDesiredLegLength] The desired leg length is not supposed to be smaller than 0." << std::endl;
        return false;
    }

    m_desiredLegLength = desiredLegLength;

    return true;
}


