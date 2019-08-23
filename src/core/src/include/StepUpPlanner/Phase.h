#ifndef STEPUPPLANNER_PHASE_H
#define STEPUPPLANNER_PHASE_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/SideDependentObject.h>
#include <StepUpPlanner/Step.h>
#include <StepUpPlanner/PhaseType.h>
#include <StepUpPlanner/State.h>
#include <StepUpPlanner/Control.h>
#include <StepUpPlanner/Rotation.h>
#include <vector>

namespace StepUpPlanner {
    class Phase;
}

class StepUpPlanner::Phase {

    casadi::DM m_duration;
    double m_minDuration, m_maxDuration, m_desiredDuration;

    StepUpPlanner::SideDependentObject<StepUpPlanner::Step> m_steps;
    StepUpPlanner::PhaseType m_phase;

    std::vector<StepUpPlanner::State> m_statesSolution;
    std::vector<StepUpPlanner::Control> m_controlsSolution;

public:

    Phase();

    Phase(StepUpPlanner::PhaseType phase);

    //Use null pointer in case the corresponding foot is not on the ground
    //Object are copied
    Phase(const StepUpPlanner::Step *left, const StepUpPlanner::Step *right);

    ~Phase();

    bool setDurationSettings(double minimumDuration, double maximumDuration, double desiredDuration);

    void setLeftPosition(double px, double py, double pz);

    void setRightPosition(double px, double py, double pz);

    StepUpPlanner::PhaseType getPhaseType() const;

    const casadi::DM& duration() const;

    casadi::DM& duration();

    double minDuration() const;

    double maxDuration() const;

    double desiredDuration() const;

    //Returning only the position. In this way, it should not be possible to change the vertices (thus the number of constraints), once the solver object is created
    casadi::DM &leftPosition();

    const casadi::DM &leftPosition() const;

    double leftPosition(size_t i) const;

    casadi::DM &rightPosition();

    const casadi::DM &rightPosition() const;

    double rightPosition(size_t i) const;

    StepUpPlanner::Rotation &leftRotation();

    const StepUpPlanner::Rotation &leftRotation() const;

    StepUpPlanner::Rotation &rightRotation();

    const StepUpPlanner::Rotation &rightRotation() const;

    const StepUpPlanner::Step& getLeftStep() const;

    const StepUpPlanner::Step& getRightStep() const;

    const std::vector<StepUpPlanner::State>& states() const;

    std::vector<StepUpPlanner::State>& states();

    const std::vector<StepUpPlanner::Control>& controls() const;

    std::vector<StepUpPlanner::Control>& controls();

};

#endif // STEPUPPLANNER_PHASE_H
