#ifndef STEPUPPLANNER_SOLVER_H
#define STEPUPPLANNER_SOLVER_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/Phase.h>
#include <StepUpPlanner/Settings.h>
#include <StepUpPlanner/References.h>
#include <StepUpPlanner/State.h>
#include <StepUpPlanner/Control.h>
#include <StepUpPlanner/Step.h>
#include <StepUpPlanner/SideDependentObject.h>
#include <vector>
#include <string>
#include <memory>

namespace StepUpPlanner {
    class Solver;
}

class StepUpPlanner::Solver {

    struct PhaseData {
        StepUpPlanner::Phase phase;
        SideDependentObject<bool> isActive;
        SideDependentObject<casadi::Function> feetConstraints;
        SideDependentObject<casadi::MX> feetConstraintsBounds;
        casadi::Function accelerationConsistencyConstraint;
        SideDependentObject<casadi::MX> feetLocationParameter;
        casadi::MX minDurationParameter, maxDurationParameter;
    };

    std::vector<PhaseData> m_phases;

    StepUpPlanner::Settings m_settings;

    casadi::Function m_integratorDynamics;

    casadi::MX m_initialStateParameter, m_desiredLegLengthParameter, m_referenceTimings, m_referenceStateParameter;

    casadi::MX m_X, m_U, m_A, m_T;

    casadi::Opti m_opti;
    std::unique_ptr<casadi::OptiSol> m_solution;

    casadi::Function getIntegratorDynamics();

    void createFeetConstraintsFunction(const std::string& name, const StepUpPlanner::Step& step,
                                       casadi::Function &outputFunction, casadi::MX &outputBounds);

    casadi::Function getAccelerationConsistencyConstraintFunction(const std::string &name, const StepUpPlanner::SideDependentObject<bool> &isActive);

    bool fillPhaseDataVector(std::vector<StepUpPlanner::Phase> phases);

    void setupProblem();

    void setParametersValue(const StepUpPlanner::State &initialState, const References &references);

    void setInitialValues(const StepUpPlanner::References& references);

public:

    Solver() = delete;

    Solver(const std::vector<StepUpPlanner::Phase>& phases, const StepUpPlanner::Settings& settings);

    Solver(const Solver& other) = delete;

    Solver(Solver&& other) = delete;

    ~Solver();

    StepUpPlanner::Phase& getPhase(size_t i);

    bool solve(const StepUpPlanner::State& initialState, const StepUpPlanner::References& references);
};

#endif // STEPUPPLANNER_SOLVER_H
