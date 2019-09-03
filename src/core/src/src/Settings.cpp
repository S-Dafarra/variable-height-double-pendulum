#include <StepUpPlanner/Settings.h>
#include <iostream>
#include <vector>
#include <algorithm>

StepUpPlanner::Settings::Settings()
    : m_maximumLegLength(1.5)
      , m_minimumLegLength(0.0)
      , m_minimumCoMHeight(0.0)
      , m_staticFrictionCoefficient(0.5)
      , m_torsionalFrictionCoefficient(0.03)
      , m_finalStateAnticipation(0.3)
      , m_phaseLength(30)
      , m_solverName("mumps")
{ }

StepUpPlanner::Settings::~Settings()
{ }

bool StepUpPlanner::Settings::setMaximumLegLength(double maxLength)
{
    if (maxLength <= 0) {
        std::cerr << "[StepUpPlanner::Settings::setMaximumLegLength] The maxLength is supposed to be a positive number." << std::endl;
        return false;
    }

    if (maxLength < m_minimumLegLength) {
        std::cerr << "[StepUpPlanner::Settings::setMaximumLegLength] The maxLength is supposed to be greater than the minimum leg length." << std::endl;
        return false;
    }


    m_maximumLegLength = maxLength;

    return true;
}

double StepUpPlanner::Settings::getMaximumLegLength() const
{
    return m_maximumLegLength;
}

bool StepUpPlanner::Settings::setMinimumLegLength(double minLength)
{
    if (minLength < 0) {
        std::cerr << "[StepUpPlanner::Settings::setMinimumLegLength] The minLength is supposed to be a non-negative number." << std::endl;
        return false;
    }

    if (minLength > m_maximumLegLength) {
        std::cerr << "[StepUpPlanner::Settings::setMinimumLegLength] The minLength is supposed to be lower than the maximum leg length." << std::endl;
        return false;
    }

    m_minimumLegLength = minLength;

    return true;
}

double StepUpPlanner::Settings::getMinimumLegLength() const
{
    return m_minimumLegLength;
}

bool StepUpPlanner::Settings::setLegLengthSettings(double minLength, double maxLength)
{
    if (maxLength <= 0) {
        std::cerr << "[StepUpPlanner::Settings::setLegLengthSettings] The maxLength is supposed to be a positive number." << std::endl;
        return false;
    }

    if (minLength < 0) {
        std::cerr << "[StepUpPlanner::Settings::setLegLengthSettings] The minLength is supposed to be a non-negative number." << std::endl;
        return false;
    }

    if (maxLength < minLength) {
        std::cerr << "[StepUpPlanner::Settings::setLegLengthSettings] The maxLength is supposed to be greater than minLength." << std::endl;
        return false;
    }

    m_maximumLegLength = maxLength;
    m_minimumLegLength = minLength;

    return true;
}

double &StepUpPlanner::Settings::minimumCoMHeight()
{
    return m_minimumCoMHeight;
}

double StepUpPlanner::Settings::minimumCoMHeight() const
{
    return m_minimumCoMHeight;
}

bool StepUpPlanner::Settings::setStaticFrictionCoefficient(double staticFrictionCoeff)
{
    if (staticFrictionCoeff < 0) {
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
    if (torsionalFrictionCoeff < 0) {
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

unsigned long &StepUpPlanner::Settings::phaseLength()
{
    return m_phaseLength;
}

unsigned long StepUpPlanner::Settings::phaseLength() const
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

bool StepUpPlanner::Settings::setIpoptLinearSolver(const std::string &solverName)
{
    std::vector<std::string> availableSolvers = {"ma27", "ma57", "ma77", "ma86", "ma97", "pardiso", "wsmp", "mumps", "custom"};

    if (std::find(availableSolvers.begin(), availableSolvers.end(), solverName) == availableSolvers.end()) {
        std::cerr << "[ERROR][StepUpPlanner::References::setIpoptLinearSolver] Unknown solver name. ";
        std::cerr << "See options at https://www.coin-or.org/Ipopt/documentation/node51.html#SECTION0001111010000000000000." <<std::endl;
        return false;
    }

    m_solverName = solverName;

    return true;
}

const std::string &StepUpPlanner::Settings::getIpoptLinearSolver() const
{
    return m_solverName;
}

unsigned long &StepUpPlanner::Settings::solverVerbosity()
{
    return m_solverVerbosity;
}

unsigned long StepUpPlanner::Settings::solverVerbosity() const
{
    return m_solverVerbosity;
}

StepUpPlanner::CostWeights &StepUpPlanner::Settings::costWeights()
{
    return m_costWeights;
}

const StepUpPlanner::CostWeights &StepUpPlanner::Settings::costWeights() const
{
    return m_costWeights;
}

