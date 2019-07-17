#ifndef STEPUPPLANNER_COSTWEIGHTS_H
#define STEPUPPLANNER_COSTWEIGHTS_H

namespace StepUpPlanner {
    class CostWeights;
}

class StepUpPlanner::CostWeights {
public:

    double durationsDifference;
    double finalStateError;
    double multipliers;
    double maxMultiplier;
    double cop;
    double controlVariations;
    double finalControl;
    double torques;

    CostWeights();
};

#endif // STEPUPPLANNER_COSTWEIGHTS_H
