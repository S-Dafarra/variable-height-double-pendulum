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
        SideDependentObject<casadi::DM> feetConstraintsBounds;
        casadi::Function accelerationConsistencyConstraint;
        SideDependentObject<casadi::MX> feetLocationParameter;
        SideDependentObject<casadi::MX> feetOrientationParameter;
        casadi::MX minDurationParameter, maxDurationParameter, feetMaxHeightParameter;
    };

    enum class SolverState {
        NOT_INITIALIZED,
        PROBLEM_SET,
        PROBLEM_SOLVED
    };

    SolverState m_solverState;

    std::vector<PhaseData> m_phases;

    StepUpPlanner::Settings m_settings;

    casadi::Function m_integratorDynamics;

    casadi::MX m_initialStateParameter, m_desiredLegLengthParameter;
    casadi::MX m_referenceTimings, m_referenceStateParameter, m_referenceControlParameter;

    casadi::MX m_X, m_U, m_A, m_T, m_uMax, m_tauMax;
    casadi::DM m_Xsol, m_Usol, m_Asol, m_Tsol;
    bool m_useMaxU, m_useMaxTau;

    casadi::Opti m_opti;

    std::unique_ptr<casadi::OptiSol> m_solution;

    casadi::DM m_linSpacedPoints;

    casadi::Function getIntegratorDynamics();

    void createFeetConstraintsFunction(const std::string& name, const StepUpPlanner::Step& step,
                                       casadi::Function &outputFunction, casadi::DM &outputBounds);

    casadi::Function getAccelerationConsistencyConstraintFunction(const std::string &name, const StepUpPlanner::SideDependentObject<bool> &isActive);

    bool fillPhaseDataVector(const std::vector<StepUpPlanner::Phase>& phases);

    void setupOpti();

    bool setupProblem(const std::vector<StepUpPlanner::Phase>& phases, const StepUpPlanner::Settings& settings);

    void setParametersValue(const StepUpPlanner::State &initialState, const References &references);

    void setInitialValues(const StepUpPlanner::References& references);

    void fillSolution();

public:

    Solver();

    Solver(const std::vector<StepUpPlanner::Phase>& phases, const StepUpPlanner::Settings& settings);

    Solver(const Solver& other) = delete;

    Solver(Solver&& other) = delete;

    ~Solver();

    bool resetProblem(const std::vector<StepUpPlanner::Phase>& phases, const StepUpPlanner::Settings& settings);

    size_t numberOfPhases() const;

    StepUpPlanner::Phase& getPhase(size_t i); //Use this to change the feet positions from one iteration to the other

    const StepUpPlanner::Phase& getPhase(size_t i) const;

    bool solve(const StepUpPlanner::State& initialState, const StepUpPlanner::References& references);

    bool getFullSolution(std::vector<StepUpPlanner::Phase>& phases) const;

    void clear();

    bool isReady() const;
};

#endif // STEPUPPLANNER_SOLVER_H
