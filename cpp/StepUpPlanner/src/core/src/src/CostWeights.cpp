#include <StepUpPlanner/CostWeights.h>

StepUpPlanner::CostWeights::CostWeights()
    : durationsDifference(0.0)
      , finalStateError(0.0)
      , multipliers(0.0)
      , cop(0.0)
      , controlVariations(0.0)
      , finalControl(0.0)
      , torques(0.0)
      , maxTorques(0.0)
{}
