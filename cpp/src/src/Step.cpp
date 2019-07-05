#include <StepUpPlanner/Step.h>
#include <cassert>
#include <iostream>
#include <cmath>

bool StepUpPlanner::Step::computeCoPConstraints(const std::vector<Vertex> &vertices)
{
    if (vertices.size() < 3) {
        // Right now we consider only feet with at least three vertices. When only two vertices are available, the CoP should be constrained on
        // a line, thus the inequality should become an equality. A possibility could be to write two inequalities,
        // but this could break the LICQ condition if not correctly handled by the solver. For the time being, this special case is not considered.
        std::cerr << "[StepUpPlanner::Step::computeCoPConstraints] The vertices are supposed to be at least three." << std::endl;
        m_copConstraints = casadi::Function();
        return false;
    }

    m_edgeConstraints.resize(vertices.size(), casadi::MX(1));
    m_copBounds = casadi::MX(static_cast<casadi_int>(vertices.size()), 1);

    casadi::MX cop(2, 1);
    double xMultiplier, yMultiplier;
    double bound;

    size_t previousIndex = vertices.size() - 1;
    size_t nextIndex = 1;
    for (size_t i = 0; i < vertices.size(); ++i) {

        xMultiplier = (vertices[i].y - vertices[previousIndex].y) / (vertices[i].x - vertices[previousIndex].x);
        yMultiplier = -1;
        bound = -vertices[previousIndex].y + vertices[previousIndex].x * (vertices[i].y - vertices[previousIndex].y) /
                (vertices[i].x - vertices[previousIndex].x);

        double nextPointValue = xMultiplier * vertices [nextIndex].x + yMultiplier * vertices [nextIndex].y;
        if (std::abs(nextPointValue - bound) < 1e-8) {
            std::cerr << "[StepUpPlanner::Step::computeCoPConstraints] The vertices with index ";
            std::cerr << previousIndex << ", " << i << ", " << nextIndex << " are aligned." << std::endl;
            m_copConstraints = casadi::Function();
            return false;
        }

        //To understand the sign of the inequality, check in which half plane the next vertex lies.
        if (nextPointValue < bound) {
            m_edgeConstraints[i] = xMultiplier * cop(0) + yMultiplier * cop(1);
            m_copBounds(i) = bound;
        } else {
            m_edgeConstraints[i] = -xMultiplier * cop(0) - yMultiplier * cop(1);
            m_copBounds(i) = -bound;
        }

        previousIndex = i;
        nextIndex++;
        if (nextIndex >= vertices.size()) {
            nextIndex = 0;
        }
    }

    m_copConstraints = casadi::Function("CoPConstraints", {cop}, m_edgeConstraints);

    assert(!m_copConstraints.is_null());

    return true;
}

StepUpPlanner::Step::Step()
    : m_position(3,1)
{ }

StepUpPlanner::Step::Step(double px, double py, double pz)
{
    setPosition(px, py, pz);
}

StepUpPlanner::Step::Step(double px, double py, double pz, const std::vector<Vertex> &vertices)
{
    setPosition(px, py, pz);
    bool ok = setVertices(vertices);
    assert(ok);
}

StepUpPlanner::Step::~Step()
{ }

void StepUpPlanner::Step::setPosition(double px, double py, double pz)
{
    m_position(0) = px;
    m_position(1) = py;
    m_position(2) = pz;
}

bool StepUpPlanner::Step::setVertices(const std::vector<Vertex> &vertices)
{
    bool ok = computeCoPConstraints(vertices);
    if (!ok) {
        std::cerr << "[StepUpPlanner::Step::Step] Failed to compute the CoP constraint function." << std::endl;
        return false;
    }
    m_footVertices = vertices;

    return true;
}

casadi::DM &StepUpPlanner::Step::position()
{
    return m_position;
}

const casadi::DM &StepUpPlanner::Step::position() const
{
    return m_position;
}

casadi::MX StepUpPlanner::Step::getCoPBounds() const
{
    return m_copBounds;
}

casadi::Function StepUpPlanner::Step::getCoPConstraintsFunction() const
{
    return m_copConstraints;
}

void StepUpPlanner::Step::clear()
{
    setPosition(0.0, 0.0, 0.0);
    m_footVertices.clear();
    m_edgeConstraints.clear();
    m_copBounds = casadi::MX();
    m_copConstraints = casadi::Function();
}
