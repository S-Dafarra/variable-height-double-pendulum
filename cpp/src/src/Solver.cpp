#include <StepUpPlanner/Solver.h>
#include <iostream>
#include <cmath>
#include <cassert>

casadi::Function StepUpPlanner::Solver::getIntegratorDynamics()
{
    casadi::MX x = casadi::MX::sym("x", 6);
    casadi::MX a = casadi::MX::sym("a", 3);
    casadi::MX dT = casadi::MX::sym("dt");

    casadi::MX p = x(casadi::Slice(0, 2));
    casadi::MX v = x(casadi::Slice(3, 5));
    casadi::MX g = casadi::MX::zeros(3,1);
    g(3) = -9.81;
    casadi::MX rhs = casadi::MX::vertcat({p + dT * v + 0.5 * casadi::MX::pow(dT,2) * (a + g),
                                          v + dT * (a + g)});

    return casadi::Function("Integrator", {x, a, dT}, {rhs});
}

void StepUpPlanner::Solver::createFeetConstraintsFunction(const std::string &name, const Step &step, casadi::Function& outputFunction, casadi::MX& outputBounds)
{
    casadi::MX footLocation = casadi::MX::sym("pFoot", 3);
    casadi::MX footControl = casadi::MX::sym("foot_control", 3);
    casadi::MX state = casadi::MX::sym("state", 6);

    casadi::MX currentPosition = state(casadi::Slice(0,2));
    casadi::MX footCoP = footControl(casadi::Slice(0,1));
    casadi::MX u = footControl(2);
    casadi::MX footCoPInWorld = casadi::MX::vertcat({footCoP, 0});
    casadi::MX forceDividedByMassAndU = (currentPosition - (footLocation + footCoPInWorld)); //Being the mass and u positive quantities,
                                                                                    // while the upperbound is 0, they don't play a role


    casadi::MX frictionBounds = casadi::MX::zeros(3,1);
    casadi::MX A = casadi::MX::horzcat({-footCoP(1), footCoP(0), 0});
    casadi::MX B = casadi::MX::horzcat({0, 0, m_settings.getTorsionalFrictionCoefficient()});
    double staticFriction = m_settings.getStaticFrictionCoefficient();
    casadi::MX staticFrictionMultiplier = casadi::MX::horzcat({1.0, 1.0, -(staticFriction * staticFriction)});

    casadi::MX frictionExpressions(3);
    frictionExpressions(0) = casadi::MX::mtimes(staticFrictionMultiplier, casadi::MX::pow(forceDividedByMassAndU, 2));
    frictionExpressions(1) = casadi::MX::mtimes(A-B, forceDividedByMassAndU);
    frictionExpressions(2) = casadi::MX::mtimes(-A-B, forceDividedByMassAndU);

    casadi::Function copConstraintsFcn = step.getCoPConstraintsFunction();
    casadi::MX copConstraintsExpressions = casadi::MX::vertcat(copConstraintsFcn(footCoP));

    double legLength = m_settings.getMaximMaximumLegLength();
    casadi::MX legLengthExpression = casadi::MX::mtimes((currentPosition - footLocation).T(), (currentPosition - footLocation));

    casadi::MX multiplierPositivityExpression = -u;

    casadi::MX constraintsExpression;
    if (copConstraintsFcn.is_null()) {

        constraintsExpression = casadi::MX::vertcat({frictionExpressions,
                                                     multiplierPositivityExpression, legLengthExpression});
        outputBounds = casadi::MX::vertcat({frictionBounds, 0, legLength * legLength});

    } else {

        constraintsExpression = casadi::MX::vertcat({frictionExpressions, copConstraintsExpressions,
                                                     multiplierPositivityExpression, legLengthExpression});
        outputBounds = casadi::MX::vertcat({frictionBounds, step.getCoPBounds(), 0, legLength * legLength});
    }

    outputFunction = casadi::Function(name, {state, footControl, footLocation}, {constraintsExpression});

}

casadi::Function StepUpPlanner::Solver::getAccelerationConsistencyConstraintFunction(const std::string &name,
                                                                                     const StepUpPlanner::SideDependentObject<bool>& isActive)
{
    casadi::MX X = casadi::MX::sym("x", 6);
    casadi::MX U = casadi::MX::sym("u", 6);
    casadi::MX A = casadi::MX::sym("a", 3);
    casadi::MX currentPosition = X(casadi::Slice(0,2));
    casadi::MX leftLocation = casadi::MX::sym("leftLocation", 3);
    casadi::MX rightLocation = casadi::MX::sym("rightLocation", 3);

    casadi::MX footCoPL = U(casadi::Slice(0,1));
    casadi::MX ul = U(2);
    casadi::MX leftCoPInWorld = casadi::MX::vertcat({footCoPL, 0});

    casadi::MX footCoPR = U(casadi::Slice(3,4));
    casadi::MX ur = U(5);
    casadi::MX rightCoPInWorld = casadi::MX::vertcat({footCoPR, 0});


    casadi::MX constraint = A;

    if (isActive.left) {
        constraint = constraint - (currentPosition - (leftLocation + leftCoPInWorld)) * ul;
    }

    if (isActive.right) {
        constraint = constraint - (currentPosition - (rightLocation + leftCoPInWorld)) * ur;
    }

    return casadi::Function(name, {X, U, A, leftLocation, rightLocation}, {constraint});
}

bool StepUpPlanner::Solver::fillPhaseDataVector(std::vector<StepUpPlanner::Phase> phases)
{
    m_phases.resize(phases.size());

    for (size_t i = 0; i < m_phases.size(); ++i) {

        m_phases[i].phase = phases[i];
        StepUpPlanner::PhaseType phaseType = phases[i].getPhaseType();

        if (phaseType == StepUpPlanner::PhaseType::UNDEFINED) {
            std::cerr << "[StepUpPlanner::Solver::fillPhaseDataVector] Undefined phase type at position " << i << "." <<std::endl;
            return false;
        }

        size_t activeConstraints = 0;

        m_phases[i].isActive.left = (phaseType == StepUpPlanner::PhaseType::DOUBLE_SUPPORT) ||
            (phaseType == StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT);

        if (m_phases[i].isActive.left) {
            activeConstraints++;
        }

        m_phases[i].isActive.right = (phaseType == StepUpPlanner::PhaseType::DOUBLE_SUPPORT) ||
            (phaseType == StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT);

        if (m_phases[i].isActive.right) {
            activeConstraints++;
        }

        m_phases[i].accelerationConsistencyConstraint = getAccelerationConsistencyConstraintFunction("Consinstency" + std::to_string(i),
                                                                                                     m_phases[i].isActive);

        if (m_phases[i].isActive.left) {
            createFeetConstraintsFunction("LeftFootConstraints" + std::to_string(i), phases[i].getLeftStep(),
                                          m_phases[i].feetConstraints.left, m_phases[i].feetConstraintsBounds.left);
        } else {
            m_phases[i].feetConstraints.left = casadi::Function();
        }

        if (m_phases[i].isActive.right) {
            createFeetConstraintsFunction("RightFootConstraints" + std::to_string(i), phases[i].getRightStep(),
                                          m_phases[i].feetConstraints.right, m_phases[i].feetConstraintsBounds.right);
        } else {
            m_phases[i].feetConstraints.right = casadi::Function();
        }

        m_phases[i].feetLocationParameter.left  = m_opti.parameter(3);
        m_phases[i].feetLocationParameter.right = m_opti.parameter(3);
        m_phases[i].minDurationParameter = m_opti.parameter();
        m_phases[i].maxDurationParameter = m_opti.parameter();
    }

    m_referenceTimings = m_opti.parameter(static_cast<casadi_int>(m_phases.size()));

    return true;
}

void StepUpPlanner::Solver::setupProblem()
{
    casadi_int numberOfPhases = static_cast<casadi_int>(m_phases.size());
    casadi_int phaseLength = static_cast<casadi_int>(m_settings.phaseLength());
    casadi_int N = numberOfPhases * phaseLength;

    m_X = m_opti.variable(6, N + 1);
    m_A = m_opti.variable(3,N);
    m_U = m_opti.variable(6, N);
    m_T = m_opti.variable(numberOfPhases);

    casadi::MX currentState = m_X(casadi::Slice(), 0);
    m_opti.subject_to(currentState == m_initialStateParameter);

    casadi::MX previousState = currentState;
    casadi::MX currentControl, currentAcceleration, leftControl, rightControl;

    casadi::MX torquesCost = 0;

    for (casadi_int phase = 0; phase < numberOfPhases; ++phase) {

        casadi::MX dT = m_T(phase)/phaseLength;

        size_t castedPhase = static_cast<size_t>(phase);
        bool leftIsActive = m_phases[castedPhase].isActive.left;
        bool rightIsActive = m_phases[castedPhase].isActive.right;

        casadi::Function leftFootConstraints = m_phases[castedPhase].feetConstraints.left;
        casadi::MX leftFootBounds = m_phases[castedPhase].feetConstraintsBounds.left;
        casadi::MX leftPosition = m_phases[castedPhase].feetLocationParameter.left;

        casadi::Function rightFootConstraints = m_phases[castedPhase].feetConstraints.right;
        casadi::MX rightFootBounds = m_phases[castedPhase].feetConstraintsBounds.right;
        casadi::MX rightPosition = m_phases[castedPhase].feetLocationParameter.right;

        casadi::Function accelerationConstraint = m_phases[castedPhase].accelerationConsistencyConstraint;


        for (casadi_int k = 0; k < phaseLength; ++k) {
            currentState = m_X(casadi::Slice(), k + 1);
            currentControl = m_U(casadi::Slice(), k);
            currentAcceleration = m_A(casadi::Slice(), k);

            leftControl = currentControl(casadi::Slice(0,2));
            rightControl = currentControl(casadi::Slice(3,5));
            m_opti.subject_to(currentState == casadi::MX::vertcat(m_integratorDynamics({previousState, currentAcceleration, dT})));

            if (leftIsActive) {
                m_opti.subject_to(casadi::MX::vertcat(leftFootConstraints({currentState, leftControl, leftPosition})) <= leftFootBounds);
                torquesCost += casadi::MX::pow(((currentState(2) - leftPosition(2) - m_desiredLegLengthParameter) * leftControl(2)), 2);
            } else {
                m_opti.subject_to(leftControl == casadi::MX::zeros(3,1));
            }

            if (rightIsActive) {
                m_opti.subject_to(casadi::MX::vertcat(rightFootConstraints({currentState, rightControl, rightPosition})) <= rightFootBounds);
                torquesCost += casadi::MX::pow(((currentState(2) - rightPosition(2) - m_desiredLegLengthParameter) * rightControl(2)), 2);
            } else {
                m_opti.subject_to(rightControl == casadi::MX::zeros(3,1));
            }

            m_opti.subject_to(casadi::MX::vertcat(accelerationConstraint({currentState, currentControl, currentAcceleration,
                                                                          leftPosition, rightPosition})) == casadi::MX::zeros(3,1));

            previousState = currentState;
        }

        m_opti.subject_to(m_phases[castedPhase].minDurationParameter <= m_T(phase) <= m_phases[castedPhase].maxDurationParameter);
    }

    StepUpPlanner::CostWeights w = m_settings.costWeights();
    casadi::Slice lastStates(static_cast<casadi_int>(N - std::round(phaseLength * m_settings.getFinalStateAnticipation())), N);

    casadi::MX costFunction = w.durationsDifference * casadi::MX::sumsqr(m_T - m_referenceTimings); //Timing error
    costFunction += w.finalStateError * casadi::MX::sumsqr(m_X(casadi::Slice(), lastStates) - m_referenceStateParameter); //Final state error

    m_opti.minimize(costFunction);
}

void StepUpPlanner::Solver::setParametersValue(const StepUpPlanner::State &initialState, const References &references)
{
    m_opti.set_value(m_initialStateParameter, casadi::DM::vertcat({initialState.position(), initialState.velocity()}));
    m_opti.set_value(m_desiredLegLengthParameter, references.getDesiredLength());
    m_opti.set_value(m_referenceStateParameter(casadi::Slice(0,2)), references.desiredState().position());
    m_opti.set_value(m_referenceStateParameter(casadi::Slice(3,5)), references.desiredState().velocity());


    for (size_t i = 0; i < m_phases.size(); ++i) {
        m_opti.set_value(m_phases[i].feetLocationParameter.left, m_phases[i].phase.leftPosition());
        m_opti.set_value(m_phases[i].feetLocationParameter.right, m_phases[i].phase.rightPosition());
        m_opti.set_value(m_phases[i].minDurationParameter, m_phases[i].phase.minDuration());
        m_opti.set_value(m_phases[i].maxDurationParameter, m_phases[i].phase.maxDuration());
        m_opti.set_value(m_referenceTimings(i), m_phases[i].phase.desiredDuration());
    }
}

StepUpPlanner::Solver::Solver(const std::vector<StepUpPlanner::Phase> &phases, const StepUpPlanner::Settings &settings)
      : m_settings(settings)
{
    m_initialStateParameter = m_opti.parameter(6);
    m_desiredLegLengthParameter = m_opti.parameter();
    m_referenceStateParameter = m_opti.parameter(6);
    m_integratorDynamics = getIntegratorDynamics();

    bool ok = fillPhaseDataVector(phases);
    assert(ok);
    setupProblem();
}

StepUpPlanner::Solver::~Solver()
{ }

StepUpPlanner::Phase &StepUpPlanner::Solver::getPhase(size_t i)
{
    assert(i < m_phases.size() && "[ERROR][StepUpPlanner::Solver::getPhase] Index out of bounds.");
    return m_phases[i].phase;
}

bool StepUpPlanner::Solver::solve(const StepUpPlanner::State &initialState, const References &references)
{
    setParametersValue(initialState, references);

    m_solution = std::make_unique<casadi::OptiSol>(m_opti.solve());

    return true;
}

