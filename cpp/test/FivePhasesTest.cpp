#include <StepUpPlanner/Solver.h>

int main() {

    StepUpPlanner::Step l1, r1, l2, r2;

    l1.setPosition(0.0, 0.15, 0.0);
    l1.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}});

    r1.setPosition(0.0, -0.15, 0.0);
    r1.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}});

    l2.setPosition(0.6, 0.15, 0.4);
    l2.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}});

    r2.setPosition(0.6, -0.15, 0.4);
    r2.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}});

    std::vector<StepUpPlanner::Phase> phases = {StepUpPlanner::Phase(&l1, &r1),
                                                StepUpPlanner::Phase(nullptr, &r1),
                                                StepUpPlanner::Phase(&l2, &r1),
                                                StepUpPlanner::Phase(&l2, nullptr),
                                                StepUpPlanner::Phase(&l2, &r2)};



    StepUpPlanner::Settings settings;

    settings.phaseLength() = 30;
    settings.solverVerbosity() = 11;
    settings.setMaximumLegLength(1.2);
    settings.setIpoptLinearSolver("ma27");
    settings.setFinalStateAnticipation(0.3);
    settings.setStaticFrictionCoefficient(0.5);
    settings.setTorsionalFrictionCoefficient(0.1);

    double N = settings.phaseLength() * phases.size();

    StepUpPlanner::CostWeights& weights = settings.costWeights();
    weights.cop = 10.0/N;
    weights.torques = 1.0/N;
    weights.multipliers = 0.1/N;
    weights.finalControl = 1.0;
    weights.maxMultiplier = 0.1;
    weights.finalStateError = 10;
    weights.controlVariations = 1.0/N;
    weights.durationsDifference = 1.0;

    StepUpPlanner::Solver solver(phases, settings);

}
