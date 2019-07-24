#include <StepUpPlanner/Settings.h>
#include <iostream>
#include <vector>
#include <algorithm>

StepUpPlanner::Settings::Settings()
    : m_maximumLegLength(1.5)
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

    m_maximumLegLength = maxLength;

    return true;
}

double StepUpPlanner::Settings::getMaximMaximumLegLength() const
{
    return m_maximumLegLength;
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

