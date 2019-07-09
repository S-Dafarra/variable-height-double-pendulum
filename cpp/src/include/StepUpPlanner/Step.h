#ifndef STEPUPPLANNER_STEP_H
#define STEPUPPLANNER_STEP_H

#include <casadi/casadi.hpp>
#include <vector>

namespace StepUpPlanner {
    struct Vertex;
    class Step;
}

struct StepUpPlanner::Vertex {
    double x;
    double y;
};

class StepUpPlanner::Step {

    casadi::DM m_position;
    std::vector<StepUpPlanner::Vertex> m_footVertices;
    casadi::MX m_edgeConstraints;

    casadi::DM m_copBounds;
    casadi::Function m_copConstraints;

    bool computeCoPConstraints(const std::vector<StepUpPlanner::Vertex>& vertices);

public:

    Step();

    Step(double px, double py, double pz);

    Step(double px, double py, double pz, const std::vector<StepUpPlanner::Vertex>& vertices);

    ~Step();

    void setPosition(double px, double py, double pz);

    bool setVertices(const std::vector<StepUpPlanner::Vertex>& vertices);

    casadi::DM &position();

    const casadi::DM &position() const;

    casadi::DM getCoPBounds() const;

    casadi::Function getCoPConstraintsFunction() const;

    void clear();

};

#endif // STEPUPPLANNER_STEP_H
