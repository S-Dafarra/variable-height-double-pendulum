#ifndef STEPUPPLANNER_SOLVER_H
#define STEPUPPLANNER_SOLVER_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/Phase.h>
#include <StepUpPlanner/Settings.h>
#include <StepUpPlanner/State.h>
#include <vector>

namespace StepUpPlanner {
    class Solver;
}

class StepUpPlanner::Solver {

    std::vector<StepUpPlanner::Phase> m_phases;
    unsigned int m_phaseLength;

    StepUpPlanner::Settings m_settings;

public:

    Solver() = delete;

    Solver(const std::vector<StepUpPlanner::Phase>& phases, unsigned int phaseLength);

    Solver(const Solver& other) = delete;

    Solver(Solver&& other) = delete;

    ~Solver();

    StepUpPlanner::Phase& getPhase(size_t i);

    void specifySettings(const StepUpPlanner::Settings& settings);

    bool solve(const StepUpPlanner::State& state);
};

#endif // STEPUPPLANNER_SOLVER_H
