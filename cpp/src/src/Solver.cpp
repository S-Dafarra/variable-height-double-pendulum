#include <StepUpPlanner/Solver.h>
#include <cassert>

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

bool StepUpPlanner::Solver::solve(const StepUpPlanner::State &state)
{
    return false;
}

