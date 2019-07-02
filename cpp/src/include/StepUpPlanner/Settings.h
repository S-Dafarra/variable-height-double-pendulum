#ifndef STEPUPPLANNER_SETTINGS_H
#define STEPUPPLANNER_SETTINGS_H

#include <StepUpPlanner/CostWeights.h>

namespace StepUpPlanner {
    class Settings;
}

class StepUpPlanner::Settings {

    double m_maximumLegLength;
    double m_staticFrictionCoefficient;
    double m_torsionalFrictionCoefficient;

    double m_finalStateAnticipation;

    unsigned int m_phaseLength;

    StepUpPlanner::CostWeights m_costWeights;

public:

    Settings();

    ~Settings();

    bool setMaximumLegLength(double maxLength);

    double getMaximMaximumLegLength() const;

    bool setStaticFrictionCoefficient(double staticFrictionCoeff);

    double getStaticFrictionCoefficient() const;

    bool setTorsionalFrictionCoefficient(double torsionalFrictionCoeff);

    double getTorsionalFrictionCoefficient() const;

    unsigned int &phaseLength();

    bool setFinalStateAnticipation(double finalStateAnticipation); //The percentage of the last phase in which the error from the desired state is considered

    double getFinalStateAnticipation() const;

    StepUpPlanner::CostWeights& costWeights();

};

#endif //  STEPUPPLANNER_SETTINGS_H
