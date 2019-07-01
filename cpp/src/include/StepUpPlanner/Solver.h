#ifndef STEPUPPLANNER_SOLVER_H
#define STEPUPPLANNER_SOLVER_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/Phase.h>
#include <StepUpPlanner/Settings.h>
#include <StepUpPlanner/References.h>
#include <StepUpPlanner/State.h>
#include <StepUpPlanner/Control.h>
#include <StepUpPlanner/Step.h>
#include <vector>
#include <string>

namespace StepUpPlanner {
    class Solver;
}

class StepUpPlanner::Solver {

    std::vector<StepUpPlanner::Phase> m_phases;
    unsigned int m_phaseLength;

    StepUpPlanner::Settings m_settings;

    void createFeetConstraintsFunction(const std::string& name, const StepUpPlanner::Step& step,
                                       casadi::Function &outputFunction, casadi::MX &outputBounds);

public:

    Solver() = delete;

    Solver(const std::vector<StepUpPlanner::Phase>& phases, unsigned int phaseLength);

    Solver(const Solver& other) = delete;

    Solver(Solver&& other) = delete;

    ~Solver();

    StepUpPlanner::Phase& getPhase(size_t i);

    void specifySettings(const StepUpPlanner::Settings& settings);

    bool solve(const StepUpPlanner::State& initialState, const StepUpPlanner::References& references);
};

#endif // STEPUPPLANNER_SOLVER_H
