#include <StepUpPlanner/Solver.h>
#include <iostream>
#include <cmath>
#include <cassert>

casadi::Function StepUpPlanner::Solver::getIntegratorDynamics()
{
    casadi::MX x = casadi::MX::sym("x", 6);
    casadi::MX a = casadi::MX::sym("a", 3);
    casadi::MX dT = casadi::MX::sym("dt");

    casadi::MX p = x(casadi::Slice(0, 3));
    casadi::MX v = x(casadi::Slice(3, 6));
    casadi::DM g = casadi::DM::zeros(3,1);
    g(2) = -9.81;
    casadi::MX rhs = casadi::MX::vertcat({p + dT * (v + 0.5 * dT * (a + g)),
                                          v + dT * (a + g)});

    return casadi::Function("Integrator", {x, a, dT}, {rhs});
}

void StepUpPlanner::Solver::createFeetConstraintsFunction(const std::string &name, const Step &step, casadi::Function& outputFunction, casadi::DM &outputBounds)
{
    casadi::MX footLocation = casadi::MX::sym("pFoot", 3);
    casadi::MX footControl = casadi::MX::sym("foot_control", 3);
    casadi::MX state = casadi::MX::sym("state", 6);

    casadi::MX currentPosition = state(casadi::Slice(0,3));
    casadi::MX footCoP = footControl(casadi::Slice(0,2));
    casadi::MX u = footControl(2);
    casadi::MX footCoPInWorld = casadi::MX::vertcat({footCoP, 0});
    casadi::MX forceDividedByMassAndU = (currentPosition - (footLocation + footCoPInWorld)); //Being the mass and u positive quantities,
                                                                                    // while the upperbound is 0, they don't play a role


    casadi::DM frictionBounds = casadi::DM::zeros(3,1);
    casadi::MX A = casadi::MX::horzcat({-footCoP(1), footCoP(0), 0});
    casadi::MX B = casadi::MX::horzcat({0, 0, m_settings.getTorsionalFrictionCoefficient()});
    double staticFriction = m_settings.getStaticFrictionCoefficient();
    casadi::DM staticFrictionMultiplier = casadi::DM::horzcat({1.0, 1.0, -(staticFriction * staticFriction)});

    casadi::MX frictionExpressions(3, 1);
    frictionExpressions(0) = casadi::MX::mtimes(staticFrictionMultiplier, casadi::MX::pow(forceDividedByMassAndU, 2));
    frictionExpressions(1) = casadi::MX::mtimes(A-B, forceDividedByMassAndU);
    frictionExpressions(2) = casadi::MX::mtimes(-A-B, forceDividedByMassAndU);

    casadi::Function copConstraintsFcn = step.getCoPConstraintsFunction();

    double legLength = m_settings.getMaximMaximumLegLength();
    casadi::MX legLengthExpression = casadi::MX::mtimes((currentPosition - footLocation).T(), (currentPosition - footLocation));

    casadi::MX multiplierPositivityExpression = -u;

    if (copConstraintsFcn.is_null()) {

        outputBounds = casadi::DM::vertcat({frictionBounds, 0, legLength * legLength});

        outputFunction = casadi::Function(name, {state, footControl, footLocation}, {frictionExpressions,
                                                                                     multiplierPositivityExpression, legLengthExpression});

    } else {
        casadi::MX copConstraintsExpressions = casadi::MX::vertcat(copConstraintsFcn(footCoP));
        outputBounds = casadi::DM::vertcat({frictionBounds, step.getCoPBounds(), 0, legLength * legLength});
        outputFunction = casadi::Function(name, {state, footControl, footLocation}, {frictionExpressions, copConstraintsExpressions,
                                                                                     multiplierPositivityExpression, legLengthExpression});
    }
}

casadi::Function StepUpPlanner::Solver::getAccelerationConsistencyConstraintFunction(const std::string &name,
                                                                                     const StepUpPlanner::SideDependentObject<bool>& isActive)
{
    casadi::MX X = casadi::MX::sym("x", 6);
    casadi::MX U = casadi::MX::sym("u", 6);
    casadi::MX A = casadi::MX::sym("a", 3);
    casadi::MX currentPosition = X(casadi::Slice(0,3));
    casadi::MX leftLocation = casadi::MX::sym("leftLocation", 3);
    casadi::MX rightLocation = casadi::MX::sym("rightLocation", 3);

    casadi::MX footCoPL = U(casadi::Slice(0,2));
    casadi::MX ul = U(2);
    casadi::MX leftCoPInWorld = casadi::MX::vertcat({footCoPL, 0});

    casadi::MX footCoPR = U(casadi::Slice(3,5));
    casadi::MX ur = U(5);
    casadi::MX rightCoPInWorld = casadi::MX::vertcat({footCoPR, 0});


    casadi::MX constraint = A;

    if (isActive.left) {
        constraint -= (currentPosition - (leftLocation + leftCoPInWorld)) * ul;
    }

    if (isActive.right) {
        constraint -= (currentPosition - (rightLocation + rightCoPInWorld)) * ur;
    }

    return casadi::Function(name, {X, U, A, leftLocation, rightLocation}, {constraint});
}

bool StepUpPlanner::Solver::fillPhaseDataVector(const std::vector<Phase> &phases)
{
    m_phases.resize(phases.size());

    for (size_t i = 0; i < m_phases.size(); ++i) {

        m_phases[i].phase = phases[i];
        StepUpPlanner::PhaseType phaseType = phases[i].getPhaseType();

        if (phaseType == StepUpPlanner::PhaseType::UNDEFINED) {
            std::cerr << "[StepUpPlanner::Solver::fillPhaseDataVector] Undefined phase type at position " << i << "." <<std::endl;
            return false;
        }

        m_phases[i].isActive.left = (phaseType == StepUpPlanner::PhaseType::DOUBLE_SUPPORT) ||
            (phaseType == StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT);

        m_phases[i].isActive.right = (phaseType == StepUpPlanner::PhaseType::DOUBLE_SUPPORT) ||
            (phaseType == StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT);

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

        m_phases[i].phase.states().resize(m_settings.phaseLength());
        m_phases[i].phase.controls().resize(m_settings.phaseLength());
    }

    m_referenceTimings = m_opti.parameter(static_cast<casadi_int>(m_phases.size()));

    return true;
}

void StepUpPlanner::Solver::setupOpti()
{
    using Sl = casadi::Slice;

    casadi_int numberOfPhases = static_cast<casadi_int>(m_phases.size());
    casadi_int phaseLength = static_cast<casadi_int>(m_settings.phaseLength());
    casadi_int N = numberOfPhases * phaseLength;

    m_X = m_opti.variable(6, N + 1);
    m_A = m_opti.variable(3,N);
    m_U = m_opti.variable(6, N);
    m_T = m_opti.variable(numberOfPhases);
    m_uMax = m_opti.variable();

    m_opti.subject_to(m_X(Sl(), 0) == m_initialStateParameter);

    casadi::MX torquesCost = 0;

    for (casadi_int phase = 0; phase < numberOfPhases; ++phase) {

        size_t castedPhase = static_cast<size_t>(phase);

        casadi::MX dT = m_T(phase)/phaseLength;

        m_opti.subject_to(m_phases[castedPhase].minDurationParameter <= m_T(phase) <= m_phases[castedPhase].maxDurationParameter);

        bool leftIsActive = m_phases[castedPhase].isActive.left;
        bool rightIsActive = m_phases[castedPhase].isActive.right;

        casadi::Function& leftFootConstraints = m_phases[castedPhase].feetConstraints.left;
        casadi::DM& leftFootBounds = m_phases[castedPhase].feetConstraintsBounds.left;
        casadi::MX& leftPosition = m_phases[castedPhase].feetLocationParameter.left;

        casadi::Function& rightFootConstraints = m_phases[castedPhase].feetConstraints.right;
        casadi::DM& rightFootBounds = m_phases[castedPhase].feetConstraintsBounds.right;
        casadi::MX& rightPosition = m_phases[castedPhase].feetLocationParameter.right;

        casadi::Function accelerationConstraint = m_phases[castedPhase].accelerationConsistencyConstraint;


        for (casadi_int k = 0 + phaseLength * phase; k < phaseLength * (phase + 1); ++k) {
            m_opti.subject_to(m_X(Sl(), k + 1) == casadi::MX::vertcat(m_integratorDynamics({m_X(Sl(), k), m_A(Sl(), k), dT})));

            if (leftIsActive) {
                m_opti.subject_to(casadi::MX::vertcat(leftFootConstraints({m_X(Sl(), k + 1), m_U(Sl(0,3), k), leftPosition})) <= leftFootBounds);
                m_opti.subject_to(m_U(2, k) - m_uMax <= 0);
                torquesCost += casadi::MX::pow(((m_X(2, k + 1) - leftPosition(2) - m_desiredLegLengthParameter) * m_U(2, k)), 2);
            } else {
                m_opti.subject_to(m_U(Sl(0,3), k) == casadi::MX::zeros(3,1));
            }

            if (rightIsActive) {
                m_opti.subject_to(casadi::MX::vertcat(rightFootConstraints({m_X(Sl(), k + 1), m_U(Sl(3,6), k), rightPosition})) <= rightFootBounds);
                m_opti.subject_to(m_U(5, k) - m_uMax <= 0);
                torquesCost += casadi::MX::pow(((m_X(2, k + 1) - rightPosition(2) - m_desiredLegLengthParameter) * m_U(5, k)), 2);
            } else {
                m_opti.subject_to(m_U(Sl(3,6), k) == casadi::MX::zeros(3,1));
            }

            m_opti.subject_to(casadi::MX::vertcat(accelerationConstraint({m_X(Sl(), k + 1), m_U(Sl(), k), m_A(Sl(), k),
                                                                          leftPosition, rightPosition})) == casadi::DM::zeros(3,1));

        }
    }

    StepUpPlanner::CostWeights w = m_settings.costWeights();
    Sl lastStates(static_cast<casadi_int>(N - std::round(phaseLength * m_settings.getFinalStateAnticipation())), N+1);

    casadi::MX costFunction = w.durationsDifference * casadi::MX::sumsqr(m_T - m_referenceTimings); //Timing error
    costFunction += w.finalStateError * casadi::MX::sumsqr(m_X(Sl(), lastStates) - m_referenceStateParameter); //Final state error
    costFunction += w.multipliers * (casadi::MX::sumsqr(m_U(2, Sl())) + casadi::MX::sumsqr(m_U(5, Sl())));
    costFunction += w.maxMultiplier * m_uMax;
    costFunction += w.cop * (casadi::MX::sumsqr(m_U(Sl(0,2), Sl())) + casadi::MX::sumsqr(m_U(Sl(3,5), Sl())));
    costFunction += w.controlVariations * (casadi::MX::sumsqr(m_U(Sl(), Sl(1, N)) - m_U(Sl(), Sl(0, N-1))));
    costFunction += w.finalControl * casadi::MX::sumsqr(m_U(Sl(), N-1) - m_referenceControlParameter);
    costFunction += w.torques * torquesCost;

    m_opti.minimize(costFunction);
}

bool StepUpPlanner::Solver::setupProblem(const std::vector<StepUpPlanner::Phase> &phases, const StepUpPlanner::Settings &settings)
{
    m_settings = settings;

    m_initialStateParameter = m_opti.parameter(6);
    m_desiredLegLengthParameter = m_opti.parameter();
    m_referenceStateParameter = m_opti.parameter(6);
    m_referenceControlParameter = m_opti.parameter(6);
    m_integratorDynamics = getIntegratorDynamics();

    bool ok = fillPhaseDataVector(phases);

    if (!ok) {
        std::cerr << "[StepUpPlanner::Solver::setupProblem] Problem when specifying the phases." <<std::endl;
        return false;
    }

    casadi_int npoints = static_cast<casadi_int>(m_phases.size() * settings.phaseLength());
    m_linSpacedPoints = casadi::DM::linspace(0, 1, npoints + 1);

    m_Xsol.resize(6, npoints +1);
    m_Usol.resize(6, npoints);
    m_Asol.resize(3, npoints);
    m_Tsol.resize(static_cast<casadi_int>(m_phases.size()), 1);

    setupOpti();

    casadi::Dict casadiOptions;
    casadi::Dict ipoptOptions;

    casadiOptions["expand"] = true;
    unsigned int solverVerbosity = m_settings.solverVerbosity();
    if (solverVerbosity) {
        casadi_int ipoptVerbosity = solverVerbosity - 1;
        ipoptOptions["print_level"] = ipoptVerbosity;
        casadiOptions["print_time"] = true;
    } else {
        ipoptOptions["print_level"] = 0;
        casadiOptions["print_time"] = false;
    }
    ipoptOptions["linear_solver"] = m_settings.getIpoptLinearSolver();

    m_opti.solver("ipopt", casadiOptions, ipoptOptions);

    m_solverState = SolverState::PROBLEM_SET;

    return true;
}

void StepUpPlanner::Solver::setParametersValue(const StepUpPlanner::State &initialState, const References &references)
{
    m_opti.set_value(m_initialStateParameter(casadi::Slice(0,3)), initialState.position());
    m_opti.set_value(m_initialStateParameter(casadi::Slice(3,6)), initialState.velocity());
    m_opti.set_value(m_desiredLegLengthParameter, references.getDesiredLength());
    m_opti.set_value(m_referenceStateParameter(casadi::Slice(0,3)), references.desiredState().position());
    m_opti.set_value(m_referenceStateParameter(casadi::Slice(3,6)), references.desiredState().velocity());
    m_opti.set_value(m_referenceControlParameter(casadi::Slice(0,2)), references.desiredControl().left().cop());
    m_opti.set_value(m_referenceControlParameter(2), references.desiredControl().left().multiplier());
    m_opti.set_value(m_referenceControlParameter(casadi::Slice(3,5)), references.desiredControl().right().cop());
    m_opti.set_value(m_referenceControlParameter(5), references.desiredControl().right().multiplier());

    for (size_t i = 0; i < m_phases.size(); ++i) {
        m_opti.set_value(m_phases[i].feetLocationParameter.left, m_phases[i].phase.leftPosition());
        m_opti.set_value(m_phases[i].feetLocationParameter.right, m_phases[i].phase.rightPosition());
        m_opti.set_value(m_phases[i].minDurationParameter, m_phases[i].phase.minDuration());
        m_opti.set_value(m_phases[i].maxDurationParameter, m_phases[i].phase.maxDuration());
        m_opti.set_value(m_referenceTimings(i), m_phases[i].phase.desiredDuration());
    }
}

void StepUpPlanner::Solver::fillSolution()
{
    m_Xsol = m_solution->value(m_X);
    m_Usol = m_solution->value(m_U);
    m_Asol = m_solution->value(m_A);
    m_Tsol = m_solution->value(m_T);

    casadi_int currentInstant = 0;

    for (size_t p = 0; p < m_phases.size(); ++p) {
        StepUpPlanner::Phase& phase = m_phases[p].phase;

        std::vector<StepUpPlanner::State>& states = phase.states();
        states.resize(m_settings.phaseLength()); //Note that memory should be already allocated when setting the phases
        std::vector<StepUpPlanner::Control>& controls = phase.controls();
        controls.resize(m_settings.phaseLength());

        phase.duration() = m_Tsol(p);

        for (size_t k = 0; k < m_settings.phaseLength(); ++k) {
            states[k].position() = m_Xsol(casadi::Slice(0,3), currentInstant + 1);
            states[k].velocity() = m_Xsol(casadi::Slice(3,6), currentInstant + 1);
            controls[k].acceleration() = m_Asol(casadi::Slice(), currentInstant);
            controls[k].left().cop() = m_Usol(casadi::Slice(0,2), currentInstant);
            controls[k].left().multiplier() = m_Usol(2, currentInstant);
            controls[k].right().cop() = m_Usol(casadi::Slice(3,5), currentInstant);
            controls[k].right().multiplier() = m_Usol(5, currentInstant);

            currentInstant++;
        }
    }
}

StepUpPlanner::Solver::Solver()
    : m_solverState(SolverState::NOT_INITIALIZED)
      , m_solution(nullptr)
{ }

StepUpPlanner::Solver::Solver(const std::vector<StepUpPlanner::Phase> &phases, const StepUpPlanner::Settings &settings)
    : m_solverState(SolverState::NOT_INITIALIZED)
      , m_solution(nullptr)

{
    setupProblem(phases, settings);
}

StepUpPlanner::Solver::~Solver()
{ }

bool StepUpPlanner::Solver::resetProblem(const std::vector<StepUpPlanner::Phase> &phases, const StepUpPlanner::Settings &settings)
{
    clear();
    return setupProblem(phases, settings);
}

size_t StepUpPlanner::Solver::numberOfPhases() const
{
    return m_phases.size();
}

StepUpPlanner::Phase &StepUpPlanner::Solver::getPhase(size_t i)
{
    assert(i < m_phases.size() && "[ERROR][StepUpPlanner::Solver::getPhase] Index out of bounds.");
    assert(m_solverState == SolverState::PROBLEM_SET && "[ERROR][StepUpPlanner::Solver::getPhase] First you have to set the problem.");
    return m_phases[i].phase;
}

const StepUpPlanner::Phase &StepUpPlanner::Solver::getPhase(size_t i) const
{
    assert(i < m_phases.size() && "[ERROR][StepUpPlanner::Solver::getPhase] Index out of bounds.");
    assert(m_solverState == SolverState::PROBLEM_SET && "[ERROR][StepUpPlanner::Solver::getPhase] First you have to set the problem.");
    return m_phases[i].phase;
}

bool StepUpPlanner::Solver::solve(const StepUpPlanner::State &initialState, const References &references)
{
    if (m_solverState == SolverState::NOT_INITIALIZED) {
        std::cerr << "[StepUpPlanner::Solver::solve] The problem is not set correctly" << std::endl;
        return false;
    }

    if (m_solverState == SolverState::PROBLEM_SOLVED) {
        assert(m_solution);
        m_opti.set_initial(m_solution->value_variables());
        m_opti.set_initial(m_opti.lam_g(), m_solution->value(m_opti.lam_g()));
    } else {
        //Generic initialization
        for (size_t i = 0; i < m_phases.size(); ++i) {
            m_opti.set_initial(m_T(i), m_phases[i].phase.desiredDuration());
        }

        casadi_int npoints = static_cast<casadi_int>(m_phases.size() * m_settings.phaseLength());

        const casadi::DM& initPos = initialState.position();
        const casadi::DM& initVel = initialState.velocity();
        const casadi::DM& refPos = references.desiredState().position();

        m_opti.set_initial(m_X(casadi::Slice(), 0), casadi::DM::vertcat({initPos, initVel}));
        casadi::DM interpolatedPosition(3, 1);

        for (casadi_int k = 1; k < npoints + 1; ++k) {
            interpolatedPosition = initPos + m_linSpacedPoints(k) * (refPos - initPos);
            m_opti.set_initial(m_X(casadi::Slice(0,3), k), interpolatedPosition);
        }
    }

    m_solverState = SolverState::PROBLEM_SET;
    m_solution = nullptr;

    setParametersValue(initialState, references);

    try {
        m_solution = std::make_unique<casadi::OptiSol>(m_opti.solve());
    } catch (std::exception &e) {
        m_opti.debug().show_infeasibilities(1e-5);
        std::cerr << "[StepUpPlanner::Solver::solve] Error while solving the optimization problem." << std::endl;
        std::cerr << "Details: " << e.what() << std::endl;
        return false;
    }

    fillSolution();

    m_solverState = SolverState::PROBLEM_SOLVED;
    return true;
}

bool StepUpPlanner::Solver::getFullSolution(std::vector<StepUpPlanner::Phase> &phases) const
{
    if (m_solverState != SolverState::PROBLEM_SOLVED) {
        std::cerr << "[StepUpPlanner::Solver::getFullSolution] No solution available." << std::endl;
        return false;
    }

    phases.resize(m_phases.size());

    for (size_t i = 0; i < m_phases.size(); ++i) {
        phases[i] = m_phases[i].phase;
    }

    return true;
}

void StepUpPlanner::Solver::clear()
{
    m_solverState = SolverState::NOT_INITIALIZED;
    m_opti = casadi::Opti();
    m_solution = nullptr;
    m_phases.clear();
    m_Xsol.clear();
    m_Usol.clear();
    m_Tsol.clear();
    m_Asol.clear();
}

