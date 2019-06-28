#ifndef STEPUPPLANNER_REFERENCES_H
#define STEPUPPLANNER_REFERENCES_H

#include <StepUpPlanner/State.h>
#include <StepUpPlanner/Control.h>

namespace StepUpPlanner {
    class References;
}

class StepUpPlanner::References {

    StepUpPlanner::State m_desiredState;
    StepUpPlanner::Control m_desiredControl;

    double m_finalStateAnticipation;
    double m_desiredLegLength;


public:

    References();

    References(const References& other);

    References(References&& other);

    ~References();

    void operator=(const References& other);

    void setDesiredState(const StepUpPlanner::State& state);

    void setDesiredControl(const StepUpPlanner::Control& control);

    bool setFinalStateAnticipation(double finalStateAnticipation); //The percentage of the last phase in which the error from the desired state is considered

    bool setDesiredLegLength(double desiredLegLength);

};


#endif // STEPUPPLANNER_REFERENCES_H
