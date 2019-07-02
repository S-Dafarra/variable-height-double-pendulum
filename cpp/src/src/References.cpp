#include <StepUpPlanner/References.h>
#include <iostream>


StepUpPlanner::References::References()
      : m_desiredLegLength(1.2)
{ }

StepUpPlanner::References::~References()
{ }

bool StepUpPlanner::References::setDesiredLegLength(double desiredLegLength)
{
    if (desiredLegLength < 0.0) {
        std::cerr << "[ERROR][StepUpPlanner::References::setDesiredLegLength] The desired leg length is not supposed to be smaller than 0." << std::endl;
        return false;
    }

    m_desiredLegLength = desiredLegLength;

    return true;
}

double StepUpPlanner::References::getDesiredLength() const
{
    return m_desiredLegLength;
}

StepUpPlanner::State &StepUpPlanner::References::desiredState()
{
    return m_desiredState;
}

const StepUpPlanner::State &StepUpPlanner::References::desiredState() const
{
    return m_desiredState;
}

StepUpPlanner::Control &StepUpPlanner::References::desiredControl()
{
    return m_desiredControl;
}

const StepUpPlanner::Control &StepUpPlanner::References::desiredControl() const
{
    return m_desiredControl;
}
