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

void StepUpPlanner::Rotation::computeQuaternion()
{
    //Taken from "Contributions au contrôle automatique de véhicules aériens"
    //PhD thesis of "Minh Duc HUA"
    //INRIA Sophia Antipolis
    //www.isir.upmc.fr/files/2009THDR2323.pdf
    //Equation 3.9 (page 101)

    //Diagonal elements used only to find the maximum
    //the furthest value from zero
    //we use this value as denominator to find the other elements
    double q0 = (static_cast<double>(m_rotation(0,0))  + static_cast<double>(m_rotation(1,1)) + static_cast<double>(m_rotation(2,2)) + 1.0);
    double q1 = (static_cast<double>(m_rotation(0,0))  - static_cast<double>(m_rotation(1,1)) - static_cast<double>(m_rotation(2,2)) + 1.0);
    double q2 = (static_cast<double>(-m_rotation(0,0)) + static_cast<double>(m_rotation(1,1)) - static_cast<double>(m_rotation(2,2)) + 1.0);
    double q3 = (static_cast<double>(-m_rotation(0,0)) - static_cast<double>(m_rotation(1,1)) + static_cast<double>(m_rotation(2,2)) + 1.0);

    if (q0 < 0.0) q0 = 0.0;
    if (q1 < 0.0) q1 = 0.0;
    if (q2 < 0.0) q2 = 0.0;
    if (q3 < 0.0) q3 = 0.0;

    if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 = std::sqrt(q0);
        q1 = (static_cast<double>(m_rotation(2,1)) - static_cast<double>(m_rotation(1,2))) / (2.0 * q0);
        q2 = (static_cast<double>(m_rotation(0,2)) - static_cast<double>(m_rotation(2,0))) / (2.0 * q0);
        q3 = (static_cast<double>(m_rotation(1,0)) - static_cast<double>(m_rotation(0,1))) / (2.0 * q0);
        q0 /= 2.0;
    } else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q1 = std::sqrt(q1);
        q0 = (static_cast<double>(m_rotation(2,1)) - static_cast<double>(m_rotation(1,2))) / (2.0 * q1);
        q2 = (static_cast<double>(m_rotation(1,0)) + static_cast<double>(m_rotation(0,1))) / (2.0 * q1);
        q3 = (static_cast<double>(m_rotation(2,0)) + static_cast<double>(m_rotation(0,2))) / (2.0 * q1);
        q1 /= 2.0;
    } else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q2 = std::sqrt(q2);
        q0 = (static_cast<double>(m_rotation(0,2)) - static_cast<double>(m_rotation(2,0))) / (2.0 * q2);
        q1 = (static_cast<double>(m_rotation(1,0)) + static_cast<double>(m_rotation(0,1))) / (2.0 * q2);
        q3 = (static_cast<double>(m_rotation(1,2)) + static_cast<double>(m_rotation(2,1))) / (2.0 * q2);
        q2 /= 2.0;
    } else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q3 = std::sqrt(q3);
        q0 = (static_cast<double>(m_rotation(1,0)) - static_cast<double>(m_rotation(0,1))) / (2.0 * q3);
        q1 = (static_cast<double>(m_rotation(2,0)) + static_cast<double>(m_rotation(0,2))) / (2.0 * q3);
        q2 = (static_cast<double>(m_rotation(1,2)) + static_cast<double>(m_rotation(2,1))) / (2.0 * q3);
        q3 /= 2.0;
    } else {
        assert(false && "[StepUpPlanner::Rotation::computeQuaternion] Quaternion numerically bad conditioned");
        return;
    }

    //Here we impose that the leftmost nonzero element of the quaternion is positive
    double eps = 1e-7;
    double sign = 1.0;
    if (q0 > eps || q0 < -eps) {
        sign = q0 > 0 ? 1.0 : -1.0;
    } else if (q1 > eps || q1 < -eps) {
        sign = q1 > 0 ? 1.0 : -1.0;
    } else if (q2 > eps || q2 < -eps) {
        sign = q2 > 0 ? 1.0 : -1.0;
    } else if (q3 > eps || q3 < -eps) {
        sign = q3 > 0 ? 1.0 : -1.0;
    }

    q0 /= sign;
    q1 /= sign;
    q2 /= sign;
    q3 /= sign;

    double quaternionNorm = std::sqrt(q0 * q0 +
                                      q1 * q1 +
                                      q2 * q2 +
                                      q3 * q3);
    m_quaternion(0)  = q0 / quaternionNorm;
    m_quaternion(1) = q1 / quaternionNorm;
    m_quaternion(2) = q2 / quaternionNorm;
    m_quaternion(3) = q3 / quaternionNorm;
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
      , m_quaternion(4,1)

{
    computeQuaternion();
}

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

