#ifndef STEPUPPLANNER_SETTINGS_H
#define STEPUPPLANNER_SETTINGS_H

#include <StepUpPlanner/CostWeights.h>
#include <string>

namespace StepUpPlanner {
    class Settings;
}

class StepUpPlanner::Settings {

    double m_maximumLegLength;
    double m_minimumLegLength;
    double m_minimumCoMHeight;
    double m_staticFrictionCoefficient;
    double m_torsionalFrictionCoefficient;

    double m_finalStateAnticipation;

    unsigned long m_phaseLength;

    std::string m_solverName;
    unsigned long m_solverVerbosity;

    StepUpPlanner::CostWeights m_costWeights;

public:

    Settings();

    ~Settings();

    bool setMaximumLegLength(double maxLength);

    double getMaximumLegLength() const;

    bool setMinimumLegLength(double minLength);

    double getMinimumLegLength() const;

    bool setLegLengthSettings(double minLength, double maxLength);

    double& minimumCoMHeight();

    double minimumCoMHeight() const;

    bool setStaticFrictionCoefficient(double staticFrictionCoeff);

    double getStaticFrictionCoefficient() const;

    bool setTorsionalFrictionCoefficient(double torsionalFrictionCoeff);

    double getTorsionalFrictionCoefficient() const;

    unsigned long &phaseLength();

    unsigned long phaseLength() const;

    bool setFinalStateAnticipation(double finalStateAnticipation); //The percentage of the last phase in which the error from the desired state is considered

    double getFinalStateAnticipation() const;

    //See https://www.coin-or.org/Ipopt/documentation/node51.html#SECTION0001111010000000000000 for options
    bool setIpoptLinearSolver(const std::string& solverName);

    const std::string& getIpoptLinearSolver() const;

    unsigned long& solverVerbosity();

    unsigned long solverVerbosity() const;

    StepUpPlanner::CostWeights& costWeights();

    const StepUpPlanner::CostWeights& costWeights() const;

};

#endif //  STEPUPPLANNER_SETTINGS_H
