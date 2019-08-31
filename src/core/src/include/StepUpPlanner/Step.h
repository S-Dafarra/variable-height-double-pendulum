#ifndef STEPUPPLANNER_STEP_H
#define STEPUPPLANNER_STEP_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/Rotation.h>
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
    StepUpPlanner::Rotation m_rotation;
    std::vector<StepUpPlanner::Vertex> m_footVertices;
    std::vector<StepUpPlanner::Vertex> m_scaledFootVertices;
    double m_scaleFactor;
    double m_xOffset, m_yOffset;
    casadi::MX m_edgeConstraints;

    casadi::DM m_copBounds;
    casadi::Function m_copConstraints;

    void scaleFootVertices(double scale, const std::vector<StepUpPlanner::Vertex>& vertices, double xOffset, double yOffset);

    bool computeCoPConstraints(const std::vector<StepUpPlanner::Vertex>& vertices);

public:

    Step();

    Step(double px, double py, double pz);

    Step(double px, double py, double pz, const std::vector<StepUpPlanner::Vertex>& vertices);

    ~Step();

    void setPosition(double px, double py, double pz);

    bool setVertices(const std::vector<StepUpPlanner::Vertex>& vertices, double scale = 1.0, double xOffset = 0.0, double yOffset = 0.0);

    const std::vector<StepUpPlanner::Vertex>& getOriginalVertices() const;

    const std::vector<StepUpPlanner::Vertex>& getScaledVertices() const;

    casadi::DM &position();

    const casadi::DM &position() const;

    StepUpPlanner::Rotation& rotation();

    const StepUpPlanner::Rotation& rotation() const;

    casadi::DM getCoPBounds() const;

    casadi::Function getCoPConstraintsFunction() const;

    void clear();

};

#endif // STEPUPPLANNER_STEP_H
