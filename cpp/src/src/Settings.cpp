#include <StepUpPlanner/Settings.h>
#include <iostream>

StepUpPlanner::Settings::Settings()
    : m_maximumLegLength(1.5)
      , m_staticFrictionCoefficient(0.5)
      , m_torsionalFrictionCoefficient(0.03)
{

}

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

StepUpPlanner::CostWeights &StepUpPlanner::Settings::costWeights()
{
    return m_costWeights;
}

StepUpPlanner::References &StepUpPlanner::Settings::references()
{
    return m_references;
}
