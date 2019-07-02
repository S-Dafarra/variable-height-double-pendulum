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

    double m_desiredLegLength;


public:

    References();

    ~References();

    bool setDesiredLegLength(double desiredLegLength);

    double getDesiredLength() const;

    StepUpPlanner::State &desiredState();

    const StepUpPlanner::State &desiredState() const;

    StepUpPlanner::Control &dediredControl();

};


#endif // STEPUPPLANNER_REFERENCES_H
