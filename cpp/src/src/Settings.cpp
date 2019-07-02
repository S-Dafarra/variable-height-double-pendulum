#include <StepUpPlanner/Settings.h>
#include <iostream>

StepUpPlanner::Settings::Settings()
    : m_maximumLegLength(1.5)
      , m_staticFrictionCoefficient(0.5)
      , m_torsionalFrictionCoefficient(0.03)
      , m_finalStateAnticipation(0.3)
      , m_phaseLength(30)
{ }

StepUpPlanner::Settings::~Settings()
{ }

bool StepUpPlanner::Settings::setMaximumLegLength(double maxLength)
{
    if (maxLength <= 0) {
        std::cerr << "[StepUpPlanner::Settings::setMaximumLegLength] The maxLength is supposed to be a positive number." << std::endl;
        return false;
    }

    m_maximumLegLength = maxLength;

    return true;
}

double StepUpPlanner::Settings::getMaximMaximumLegLength() const
{
    return m_maximumLegLength;
}

bool StepUpPlanner::Settings::setStaticFrictionCoefficient(double staticFrictionCoeff)
{
    if (staticFrictionCoeff <= 0) {
        std::cerr << "[StepUpPlanner::Settings::setStaticFrictionCoefficient] The staticFrictionCoeff is supposed to be a positive number." << std::endl;
        return false;
    }

    m_staticFrictionCoefficient = staticFrictionCoeff;

    return true;
}

double StepUpPlanner::Settings::getStaticFrictionCoefficient() const
{
    return m_staticFrictionCoefficient;
}

bool StepUpPlanner::Settings::setTorsionalFrictionCoefficient(double torsionalFrictionCoeff)
{
    if (torsionalFrictionCoeff <= 0) {
        std::cerr << "[StepUpPlanner::Settings::setTorsionalFrictionCoefficient] The torsionalFrictionCoeff is supposed to be a positive number." << std::endl;
        return false;
    }

    m_torsionalFrictionCoefficient = torsionalFrictionCoeff;

    return true;
}

double StepUpPlanner::Settings::getTorsionalFrictionCoefficient() const
{
    return m_torsionalFrictionCoefficient;
}

unsigned int &StepUpPlanner::Settings::phaseLength()
{
    return m_phaseLength;
}

bool StepUpPlanner::Settings::setFinalStateAnticipation(double finalStateAnticipation)
{
    if ((finalStateAnticipation > 1.0) || (finalStateAnticipation < 0.0))
    {
        std::cerr << "[ERROR][StepUpPlanner::References::setFinalStateAnticipation] The final state anticipation is supposed to be in the interval [0.0, 1.0]." << std::endl;
        return false;
    }

    m_finalStateAnticipation = finalStateAnticipation;

    return true;
}

double StepUpPlanner::Settings::getFinalStateAnticipation() const
{
    return m_finalStateAnticipation;
}

StepUpPlanner::CostWeights &StepUpPlanner::Settings::costWeights()
{
    return m_costWeights;
}

