#ifndef STEPUPPLANNER_PHASE_H
#define STEPUPPLANNER_PHASE_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/SideDependentObject.h>
#include <StepUpPlanner/Step.h>
#include <StepUpPlanner/PhaseType.h>

namespace StepUpPlanner {
    class Phase;
}

class StepUpPlanner::Phase {

    casadi::MX m_duration;
    double m_minDuration, m_maxDuration, m_desiredDuration;

    StepUpPlanner::SideDependentObject<StepUpPlanner::Step> m_steps;
    StepUpPlanner::PhaseType m_phase;

public:

    Phase(StepUpPlanner::PhaseType phase);

    //Use null pointer in case the corresponding foot is not on the ground
    Phase(StepUpPlanner::Step *left, StepUpPlanner::Step *right);

    ~Phase();

    bool setDurationSettings(double minimumDuration, double maximumDuration, double desiredDuration);

    void setDesiredLeftPosition(double px, double py, double pz);

    void setDesiredRightPosition(double px, double py, double pz);

    StepUpPlanner::PhaseType getPhase() const;

    casadi::MX& duration();

    StepUpPlanner::Step& leftStep();

    StepUpPlanner::Step& rightStep();

};

#endif // STEPUPPLANNER_PHASE_H
