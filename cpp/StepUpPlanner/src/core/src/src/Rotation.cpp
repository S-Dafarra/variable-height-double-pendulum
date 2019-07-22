#include <StepUpPlanner/Rotation.h>
#include <cmath>
#include <cassert>

casadi::DM StepUpPlanner::Rotation::skew(double i, double j, double k)
{
    casadi::DM skewBuffer = casadi::DM::zeros(3,3);
    skewBuffer(0, 1) = -k;
    skewBuffer(0, 2) =  j;
    skewBuffer(1, 0) =  k;
    skewBuffer(1, 2) = -i;
    skewBuffer(2, 0) = -j;
    skewBuffer(2, 1) =  i;

    return skewBuffer;
}

StepUpPlanner::Rotation::Rotation()
    : m_rotation(3,3)
      , m_quaternion(4,1)
{
    m_rotation = casadi::DM::zeros(3,3);
    m_rotation(0,0) = 1.0;
    m_rotation(1,1) = 1.0;
    m_rotation(2,2) = 1.0;
}

StepUpPlanner::Rotation::Rotation(const casadi::DM &matrix)
    : m_rotation(matrix)
{ }

void StepUpPlanner::Rotation::setFromQuaternion(double realPart, double i, double j, double k)
{
    double module = std::sqrt(realPart * realPart + i * i + j * j + k * k);

    assert(module > 0.01 &&
           "[StepUpPlanner::Rotation::setFromQuaternion] The module of the specified quaternion appear to be close to zero.");

    double normalizedModule = realPart/module;
    double normalized_i = i/module;
    double normalized_j = j/module;
    double normalized_k = k/module;

    casadi::DM skewBuffer = skew(normalized_i, normalized_j, normalized_k);

    casadi::DM identity = casadi::DM::zeros(3,3);
    identity(0,0) = 1.0;
    identity(1,1) = 1.0;
    identity(2,2) = 1.0;

    m_rotation = identity + 2 * normalizedModule * skewBuffer + 2 * casadi::DM::mtimes(skewBuffer, skewBuffer);
    m_quaternion(0) = normalizedModule;
    m_quaternion(1) = normalized_i;
    m_quaternion(2) = normalized_j;
    m_quaternion(3) = normalized_k;
}

const casadi::DM &StepUpPlanner::Rotation::asMatrix() const
{
    return m_rotation;
}

const casadi::DM &StepUpPlanner::Rotation::asQuaternion() const
{
    return m_quaternion;
}

double StepUpPlanner::Rotation::asQuaternion(size_t i) const
{
    assert(i < 4);
    return static_cast<double>(m_quaternion(i));
}

StepUpPlanner::Rotation StepUpPlanner::Rotation::Identity()
{
    casadi::DM identity = casadi::DM::zeros(3,3);
    identity(0,0) = 1.0;
    identity(1,1) = 1.0;
    identity(2,2) = 1.0;

    return StepUpPlanner::Rotation(identity);
}

