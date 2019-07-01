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
    //Object are copied
    Phase(const StepUpPlanner::Step *left, const StepUpPlanner::Step *right);

    ~Phase();

    bool setDurationSettings(double minimumDuration, double maximumDuration, double desiredDuration);

    void setDesiredLeftPosition(double px, double py, double pz);

    void setDesiredRightPosition(double px, double py, double pz);

    StepUpPlanner::PhaseType getPhase() const;

    casadi::MX& duration();

    //Returning only the position. In this way, it should not be possible to change the vertices (thus the number of constraints), once the solver object is created
    casadi::MX& leftPosition();

    casadi::MX& rightPosition();

    const StepUpPlanner::Step& getLeftStep() const;

    const StepUpPlanner::Step& getRightStep() const;

};

#endif // STEPUPPLANNER_PHASE_H
