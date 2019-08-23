#include <StepUpPlanner/Rotation.h>
#include <stdlib.h>
#include <cassert>
#include <cmath>

#define ASSERT_IS_TRUE(prop) assertTrue(prop,__FILE__,__LINE__)

void assertTrue(bool prop, std::string file, int line)
{
    if( !prop )
    {
        std::cerr << file << ":" << line << " : assertTrue failure" << std::endl;
        assert(false);
        exit(EXIT_FAILURE);
    }
}

double getRandomDouble(double min = 0.0, double max = 1.0)
{
    return min + (max-min)*(static_cast<double>(rand()))/(static_cast<double>(RAND_MAX));
}

bool matricesAreEqual(const casadi::DM& left, const casadi::DM& right, double tol = 1e-15) {
    if ((left.rows() != right.rows()) || (left.columns() != right.columns())) {
        return false;
    }

    for (int i = 0; i < left.rows(); ++i) {
        for (int j = 0; j < left.columns(); ++j) {
            if (std::abs(static_cast<double>(left(i,j) - right(i,j))) > tol) {
                return false;
            }
        }
    }

    return true;
}

bool vectorsAreEqual(const casadi::DM& left, const casadi::DM& right, double tol = 1e-15) {
    if (left.rows() != right.rows()) {
        return false;
    }

    for (int i = 0; i < left.rows(); ++i) {
        if (std::abs(static_cast<double>(left(i) - right(i))) > tol) {
            return false;
        }
    }

    return true;
}

int main() {

    casadi::DM identity = casadi::DM::zeros(3,3);
    identity(0,0) = 1.0;
    identity(1,1) = 1.0;
    identity(2,2) = 1.0;

    StepUpPlanner::Rotation testRotation;

    ASSERT_IS_TRUE(matricesAreEqual(testRotation.asMatrix(), identity));

    ASSERT_IS_TRUE(matricesAreEqual(StepUpPlanner::Rotation::Identity().asMatrix(), identity));

    testRotation.setFromQuaternion(1.0, 0.0, 0.0, 0.0);

    ASSERT_IS_TRUE(matricesAreEqual(testRotation.asMatrix(), identity));

    casadi::DM quaternion(4,1);
    quaternion(0) = getRandomDouble();
    quaternion(1) = getRandomDouble(-1.0, 1.0);
    quaternion(2) = getRandomDouble(-1.0, 1.0);
    quaternion(3) = getRandomDouble(-1.0, 1.0);

    testRotation.setFromQuaternion(static_cast<double>(quaternion(0)), static_cast<double>(quaternion(1)),
                                   static_cast<double>(quaternion(2)), static_cast<double>(quaternion(3)));

    casadi::DM expectedIdentity = casadi::DM::mtimes(testRotation.asMatrix(), testRotation.asMatrix().T());

    ASSERT_IS_TRUE(matricesAreEqual(expectedIdentity, identity));

    StepUpPlanner::Rotation invertedRotation;

    invertedRotation.setFromQuaternion(-static_cast<double>(quaternion(0)), static_cast<double>(quaternion(1)),
                                   static_cast<double>(quaternion(2)), static_cast<double>(quaternion(3)));

    expectedIdentity = casadi::DM::mtimes(testRotation.asMatrix(), invertedRotation.asMatrix());

    ASSERT_IS_TRUE(matricesAreEqual(expectedIdentity, identity));

    quaternion(0) = getRandomDouble();
    quaternion(1) = getRandomDouble(-1.0, 1.0);
    quaternion(2) = getRandomDouble(-1.0, 1.0);
    quaternion(3) = getRandomDouble(-1.0, 1.0);

    testRotation.setFromQuaternion(static_cast<double>(quaternion(0)), static_cast<double>(quaternion(1)),
                                   static_cast<double>(quaternion(2)), static_cast<double>(quaternion(3)));

    StepUpPlanner::Rotation testQuaternion(testRotation.asMatrix());

    ASSERT_IS_TRUE(vectorsAreEqual(testRotation.asQuaternion(), testQuaternion.asQuaternion()));

    return EXIT_SUCCESS;
}
