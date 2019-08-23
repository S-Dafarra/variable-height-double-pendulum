#include <StepUpPlanner/Step.h>
#include <cassert>
#include <iostream>
#include <cmath>

void StepUpPlanner::Step::scaleFootVertices(double scale, const std::vector<StepUpPlanner::Vertex> &vertices)
{
    if (vertices.empty()) {
        return;
    }

    m_scaledFootVertices.resize(vertices.size());

    StepUpPlanner::Vertex footCenter;

    footCenter.x = 0.0;
    footCenter.y = 0.0;

    for (auto& vertex : vertices) {
        footCenter.x += vertex.x;
        footCenter.y += vertex.y;
    }
    footCenter.x /= vertices.size();
    footCenter.y /= vertices.size();

    double k_x = footCenter.x * (1.0 - scale);
    double k_y = footCenter.y * (1.0 - scale);

    for (size_t i = 0; i < vertices.size(); ++i) {
        m_scaledFootVertices[i].x = k_x + scale * vertices[i].x;
        m_scaledFootVertices[i].y = k_y + scale * vertices[i].y;
    }

}

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

    m_edgeConstraints = casadi::MX(static_cast<casadi_int>(vertices.size()), 1);
    m_copBounds = casadi::DM(static_cast<casadi_int>(vertices.size()), 1);

    casadi::MX cop = casadi::MX::sym("cop", 2, 1);
    double xMultiplier, yMultiplier;
    double bound;

    size_t previousIndex = vertices.size() - 1;
    size_t nextIndex = 1;
    for (size_t i = 0; i < vertices.size(); ++i) {

        if (std::abs(vertices[i].y - vertices[previousIndex].y) < 1e-8 && std::abs(vertices[i].x - vertices[previousIndex].x) < 1e-8) {
            std::cerr << "[StepUpPlanner::Step::computeCoPConstraints] Vertex " << i << " and " << previousIndex;
            std::cerr << "seem to be coincident." << std::endl;
            m_copConstraints = casadi::Function();
            return false;
        }

        if (std::abs(vertices[i].x - vertices[previousIndex].x) > std::abs(vertices[i].y - vertices[previousIndex].y)) {
            xMultiplier = (vertices[i].y - vertices[previousIndex].y) / (vertices[i].x - vertices[previousIndex].x);
            yMultiplier = -1;
            bound = -vertices[previousIndex].y + vertices[previousIndex].x * (vertices[i].y - vertices[previousIndex].y) /
                    (vertices[i].x - vertices[previousIndex].x);
        } else {
            xMultiplier = -1;
            yMultiplier = (vertices[i].x - vertices[previousIndex].x) / (vertices[i].y - vertices[previousIndex].y);
            bound = -vertices[previousIndex].x + vertices[previousIndex].y * (vertices[i].x - vertices[previousIndex].x) /
                    (vertices[i].y - vertices[previousIndex].y);
        }


        double nextPointValue = xMultiplier * vertices [nextIndex].x + yMultiplier * vertices [nextIndex].y;
        if (std::abs(nextPointValue - bound) < 1e-8) {
            std::cerr << "[StepUpPlanner::Step::computeCoPConstraints] The vertices with index ";
            std::cerr << previousIndex << ", " << i << ", " << nextIndex << " are aligned." << std::endl;
            m_copConstraints = casadi::Function();
            return false;
        }

        //To understand the sign of the inequality, check in which half plane the next vertex lies.
        if (nextPointValue < bound) {
            m_edgeConstraints(i) = xMultiplier * cop(0) + yMultiplier * cop(1);
            m_copBounds(i) = bound;
        } else {
            m_edgeConstraints(i) = -xMultiplier * cop(0) - yMultiplier * cop(1);
            m_copBounds(i) = -bound;
        }

        previousIndex = i;
        nextIndex++;
        if (nextIndex >= vertices.size()) {
            nextIndex = 0;
        }
    }

    m_copConstraints = casadi::Function("CoPConstraints", {cop}, {m_edgeConstraints});

    assert(!m_copConstraints.is_null());

    return true;
}

StepUpPlanner::Step::Step()
    : m_position(3,1)
      , m_scaleFactor(1.0)
{ }

StepUpPlanner::Step::Step(double px, double py, double pz)
    : m_position(3,1)
      , m_scaleFactor(1.0)
{
    setPosition(px, py, pz);
}

StepUpPlanner::Step::Step(double px, double py, double pz, const std::vector<Vertex> &vertices)
    : m_position(3,1)
      , m_scaleFactor(1.0)
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

bool StepUpPlanner::Step::setVertices(const std::vector<Vertex> &vertices, double scale)
{
    if (scale < 0.0) {
        std::cerr << "[StepUpPlanner::Step::Step] The scale is supposed to be positive." << std::endl;
        return false;
    }

    scaleFootVertices(scale, vertices);

    bool ok = computeCoPConstraints(m_scaledFootVertices);
    if (!ok) {
        std::cerr << "[StepUpPlanner::Step::Step] Failed to compute the CoP constraint function." << std::endl;
        scaleFootVertices(m_scaleFactor, vertices);
        return false;
    }
    m_scaleFactor = scale;
    m_footVertices = vertices;

    return true;
}

const std::vector<StepUpPlanner::Vertex> &StepUpPlanner::Step::getOriginalVertices() const
{
    return m_footVertices;
}

const std::vector<StepUpPlanner::Vertex> &StepUpPlanner::Step::getScaledVertices() const
{
    return m_scaledFootVertices;
}

casadi::DM &StepUpPlanner::Step::position()
{
    return m_position;
}

const casadi::DM &StepUpPlanner::Step::position() const
{
    return m_position;
}

StepUpPlanner::Rotation &StepUpPlanner::Step::rotation()
{
    return m_rotation;
}

const StepUpPlanner::Rotation &StepUpPlanner::Step::rotation() const
{
    return m_rotation;
}

casadi::DM StepUpPlanner::Step::getCoPBounds() const
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
    m_scaledFootVertices.clear();
    m_scaleFactor = 1.0;
    m_edgeConstraints = casadi::MX();
    m_copBounds = casadi::DM();
    m_copConstraints = casadi::Function();
}
