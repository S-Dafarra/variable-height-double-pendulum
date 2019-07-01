#ifndef STEPUPPLANNER_SETTINGS_H
#define STEPUPPLANNER_SETTINGS_H

#include <StepUpPlanner/CostWeights.h>
#include <StepUpPlanner/References.h>

namespace StepUpPlanner {
    class Settings;
}

class StepUpPlanner::Settings {

    double m_maximumLegLength;
    double m_staticFrictionCoefficient;
    double m_torsionalFrictionCoefficient;

    StepUpPlanner::CostWeights m_costWeights;
    StepUpPlanner::References m_references;

public:

    Settings();

    ~Settings();

    bool setMaximumLegLength(double maxLength);

    double getMaximMaximumLegLength() const;

    bool setStaticFrictionCoefficient(double staticFrictionCoeff);

    double getStaticFrictionCoefficient() const;

    bool setTorsionalFrictionCoefficient(double torsionalFrictionCoeff);

    double getTorsionalFrictionCoefficient() const;

    StepUpPlanner::CostWeights& costWeights();

    StepUpPlanner::References& references();

};

#endif //  STEPUPPLANNER_SETTINGS_H
