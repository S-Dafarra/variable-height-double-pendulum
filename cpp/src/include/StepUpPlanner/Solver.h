#ifndef STEPUPPLANNER_SOLVER_H
#define STEPUPPLANNER_SOLVER_H

#include <casadi/casadi.hpp>
#include <StepUpPlanner/Phase.h>

#include <vector>

namespace StepUpPlanner {
    class Solver;
}

class StepUpPlanner::Solver {

    std::vector<StepUpPlanner::Phase> m_phases;
    unsigned int m_phaseLength;

public:

    Solver() = delete;

    Solver(const std::vector<StepUpPlanner::Phase>& phases, unsigned int phaseLength);

    Solver(const Solver& other) = delete;

    Solver(Solver&& other) = delete;

    ~Solver();

    StepUpPlanner::Phase& getPhase(size_t i);

    bool solve();
};

#endif // STEPUPPLANNER_SOLVER_H
