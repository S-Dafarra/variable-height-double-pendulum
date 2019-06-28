#ifndef STEPUPPLANNER_PHASE_H
#define STEPUPPLANNER_PHASE_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/PhaseType.h>

namespace StepUpPlanner {
    class Phase;
}

class StepUpPlanner::Phase {

    casadi::MX m_duration;
    double m_minDuration, m_maxDuration, m_desiredDuration;

    casadi::MX m_leftPosition, m_rightPosition;
    StepUpPlanner::PhaseType m_phase;

public:

    Phase(StepUpPlanner::PhaseType phase);

    ~Phase();

    bool setDurationSettings(double minimumDuration, double maximumDuration, double desiredDuration);

    void setDesiredLeftPosition(double px, double py, double pz);

    void setDesiredRightPosition(double px, double py, double pz);

    StepUpPlanner::PhaseType getPhase() const;

    casadi::MX& getDuration();

    casadi::MX& getLeftPosition();

    casadi::MX& getRightPosition();

};

#endif // STEPUPPLANNER_PHASE_H
