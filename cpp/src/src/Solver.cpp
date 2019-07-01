#include <StepUpPlanner/Solver.h>
#include <cmath>
#include <cassert>

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

    casadi::MX legLengthExpression = casadi::MX::mtimes((currentPosition - footLocation).T(), (currentPosition - footLocation));

    casadi::MX multiplierPositivityExpression = -u;

    casadi::MX constraintsExpression = casadi::MX::vertcat({frictionExpressions, copConstraintsExpressions,
                                                            multiplierPositivityExpression, legLengthExpression});

    outputFunction = casadi::Function(name, {state, footControl, footLocation}, {constraintsExpression});

    double legLength = m_settings.getMaximMaximumLegLength();

    outputBounds = casadi::MX::vertcat({frictionBounds, step.getCoPBounds(), 0, legLength * legLength});
}

StepUpPlanner::Solver::Solver(const std::vector<StepUpPlanner::Phase> &phases, unsigned int phaseLength)
    : m_phases(phases)
      , m_phaseLength(phaseLength)
{

}

StepUpPlanner::Solver::~Solver()
{ }

StepUpPlanner::Phase &StepUpPlanner::Solver::getPhase(size_t i)
{
    assert(i < m_phases.size() && "[ERROR][StepUpPlanner::Solver::getPhase] Index out of bounds.");
    return m_phases[i];
}

void StepUpPlanner::Solver::specifySettings(const Settings &settings)
{
    m_settings = settings;
}

bool StepUpPlanner::Solver::solve(const StepUpPlanner::State &initialState, const References &references)
{
    return false;
}

