#include <StepUpPlanner/Solver.h>
#include <StepUpPlanner/Plotter.h>
#include <cassert>
#include <future>
#include <thread>

#define ASSERT_IS_TRUE(prop) assertTrue(prop,__FILE__,__LINE__)

void assertTrue(bool prop, std::string file, int line)
{
    if( !prop )
    {
        std::cerr << file << ":" << line << " : assertTrue failure" << std::endl;
        assert(false);
        exit(EXIT_FAILURE);
    }
}

int main() {

    StepUpPlanner::Step l1, r1, l2, r2;

    l1.setPosition(0.0, 0.15, 0.0);
    l1.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}}, 0.9);
    ASSERT_IS_TRUE(!l1.getCoPConstraintsFunction().is_null());

    r1.setPosition(0.0, -0.15, 0.0);
    r1.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}}, 0.9);
    ASSERT_IS_TRUE(!r1.getCoPConstraintsFunction().is_null());


    l2.setPosition(0.6, 0.15, 0.4);
    l2.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}}, 0.9);
    ASSERT_IS_TRUE(!l2.getCoPConstraintsFunction().is_null());


    r2.setPosition(0.6, -0.15, 0.4);
    r2.setVertices({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}}, 0.9);
    ASSERT_IS_TRUE(!r2.getCoPConstraintsFunction().is_null());


    std::vector<StepUpPlanner::Phase> phases = {StepUpPlanner::Phase(&l1, &r1),
                                                StepUpPlanner::Phase(nullptr, &r1),
                                                StepUpPlanner::Phase(&l2, &r1),
                                                StepUpPlanner::Phase(&l2, nullptr),
                                                StepUpPlanner::Phase(&l2, &r2)};

    phases[0].setDurationSettings(0.5, 2.0, 0.8);
    phases[1].setDurationSettings(0.5, 2.0, 1.2);
    phases[2].setDurationSettings(0.5, 2.0, 0.8);
    phases[3].setDurationSettings(0.5, 2.0, 1.2);
    phases[4].setDurationSettings(0.5, 2.0, 0.8);

    StepUpPlanner::Settings settings;

    settings.phaseLength() = 30;
    settings.solverVerbosity() = 1;
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
    weights.durationsDifference = 5.0/phases.size();
    weights.maxTorques = 0.1;

    StepUpPlanner::Solver solver(phases, settings);

    StepUpPlanner::State initialState;
    initialState.setPosition(0.0, 0.0, 1.16);
    initialState.setVelocity(0.0, 0.0, 0.0);

    StepUpPlanner::References references;
    references.zero();
    references.desiredState().setPosition(0.6, 0.0, 1.56);
    references.desiredState().setVelocity(0.0, 0.0, 0.0);

    references.desiredControl().zero();
    double desiredLeftMultiplier = static_cast<double>(9.81/(2.0*(references.desiredState().position()(2) - l2.position()(2))));
    references.desiredControl().left().setMultiplier(desiredLeftMultiplier);
    double desiredRightMultiplier = static_cast<double>(9.81/(2.0*(references.desiredState().position()(2) - r2.position()(2))));
    references.desiredControl().right().setMultiplier(desiredRightMultiplier);

    bool ok = references.setDesiredLegLength(1.18);
    ASSERT_IS_TRUE(ok);

    ok = solver.solve(initialState, references);
    ASSERT_IS_TRUE(ok);

    ok = solver.getFullSolution(phases);
    ASSERT_IS_TRUE(ok);


    StepUpPlanner::Plotter plotter;


    ok = solver.solve(initialState, references);
    ASSERT_IS_TRUE(ok);

    ok = solver.getFullSolution(phases);
    ASSERT_IS_TRUE(ok);

    plotter.plotFullSolution(phases);

    std::future<int> future = std::async(std::launch::async, getchar);

    std::cerr << "Press a button to continue..." << std::endl;
    while (future.wait_for(std::chrono::microseconds(1)) != std::future_status::ready) {
        plotter.drawAll();
    }

    std::cerr << "Closing figs." << std::endl;
    plotter.closeAll();

    //reset and test the three phases

    phases = {StepUpPlanner::Phase(nullptr, &r1),
              StepUpPlanner::Phase(&l2, &r1),
              StepUpPlanner::Phase(&l2, nullptr)};

    phases[0].setDurationSettings(0.5, 2.0, 1.2);
    phases[1].setDurationSettings(0.5, 2.0, 0.8);
    phases[2].setDurationSettings(0.5, 2.0, 1.2);

    initialState.zero();
    initialState.setPosition(0.0, -0.08, 1.16);
    initialState.setVelocity(0.1, -0.05, 0.0);
    references.desiredState().setPosition(0.6, 0.0, 1.56);
    references.desiredState().setVelocity(0.0, 0.0, 0.0);

    references.desiredControl().zero();
    desiredLeftMultiplier = static_cast<double>(9.81/((references.desiredState().position()(2) - l2.position()(2))));
    references.desiredControl().left().setMultiplier(desiredLeftMultiplier);

    ok = solver.resetProblem(phases, settings);
    ASSERT_IS_TRUE(ok);

    ok = solver.solve(initialState, references);
    ASSERT_IS_TRUE(ok);

    ok = solver.getFullSolution(phases);
    ASSERT_IS_TRUE(ok);

    ok = solver.solve(initialState, references);
    ASSERT_IS_TRUE(ok);

    ok = solver.getFullSolution(phases);
    ASSERT_IS_TRUE(ok);

    plotter.plotFullSolution(phases);
    future = std::async(std::launch::async, getchar);

    std::cerr << "Press a button to continue..." << std::endl;
    while (future.wait_for(std::chrono::microseconds(1)) != std::future_status::ready) {
        plotter.drawAll();
    }
    std::cerr << "Closing figs." << std::endl;

    plotter.closeAll();

    return EXIT_SUCCESS;
}
