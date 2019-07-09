#include <StepUpPlanner/Plotter.h>
#include <matplotlibcpp/matplotlibcpp.h>

template<typename EvaluationLambda>
void ResizeAndFillVector(size_t size, const EvaluationLambda& lambda, std::vector<double>& output) {

    output.resize(size);

    for (size_t i = 0; i < size; ++i) {
        output[i] = lambda(i);
    }
}

template<typename EvaluationLambda>
void ResizeAndFill2Dplot(size_t size, const EvaluationLambda& lambda,
                         StepUpPlanner::Plot& plot)
{
    ResizeAndFillVector(size, [lambda](size_t i){return lambda(i, 0);}, plot["x"]);
    ResizeAndFillVector(size, [lambda](size_t i){return lambda(i, 1);}, plot["y"]);
}

template<typename EvaluationLambda>
void ResizeAndFill3Dplot(size_t size, const EvaluationLambda& lambda,
                         StepUpPlanner::Plot& plot)
{
    ResizeAndFill2Dplot(size, lambda, plot);
    ResizeAndFillVector(size, [lambda](size_t i){return lambda(i, 2);}, plot["z"]);
}


StepUpPlanner::Plot::Plot()
    : m_figureNumber(-1)
{ }

std::vector<double> &StepUpPlanner::Plot::operator[](const std::string &key)
{
    return m_data[key];
}

std::unordered_map<std::string, std::vector<double> > &StepUpPlanner::Plot::data()
{
    return m_data;
}

const std::unordered_map<std::string, std::vector<double> > &StepUpPlanner::Plot::data() const
{
    return m_data;
}

long &StepUpPlanner::Plot::figureNumber()
{
    return m_figureNumber;
}

std::string &StepUpPlanner::Plot::xLabel()
{
    return m_xLabel;
}

std::string &StepUpPlanner::Plot::yLabel()
{
    return m_yLabel;
}

size_t StepUpPlanner::Plotter::getNumberOfPoints(const std::vector<StepUpPlanner::Phase> &phases)
{
    size_t points = 0;

    for (const StepUpPlanner::Phase& phase : phases) {
        points += phase.states().size();
    }

    return points;
}

void StepUpPlanner::Plotter::setTimeVector(const std::vector<StepUpPlanner::Phase> &phases)
{
    m_time.resize(getNumberOfPoints(phases));
    m_phaseTimings.resize(phases.size());

    size_t index = 0;
    double previousTime = 0.0;
    double previousPhaseTime = 0.0;

    for (size_t p =0; p < phases.size(); ++p) {
        const StepUpPlanner::Phase& phase = phases[p];
        double dT = static_cast<double>(phase.duration()) / phase.states().size();

        m_phaseTimings[p] = previousPhaseTime + static_cast<double>(phase.duration());
        previousPhaseTime = m_phaseTimings[p];

        for (size_t i = 0; i < phase.states().size(); ++i) {
            m_time[index] = previousTime + dT;
            previousTime = m_time[index];
            index++;
        }
    }
}

void StepUpPlanner::Plotter::fillPlotsData(const std::vector<StepUpPlanner::Phase> &phases)
{
    if (!phases.size()) {
        return;
    }

    setTimeVector(phases);

    size_t n = m_time.size();
    size_t phaseLength = phases.begin()->states().size();

    ResizeAndFill3Dplot(n, [phases, phaseLength](size_t i, size_t j)
                        {return phases[i / phaseLength].states()[i % phaseLength].position(j);},
                        m_plots["CoM Position"]);
    m_plots["CoM Position"].xLabel() = "t [s]";
    m_plots["CoM Position"].yLabel() = "[m]";

    ResizeAndFill3Dplot(n, [phases, phaseLength](size_t i, size_t j)
                        {return phases[i / phaseLength].states()[i % phaseLength].velocity(j);},
                        m_plots["CoM Velocity"]);

    m_plots["CoM Velocity"].xLabel() = "t [s]";
    m_plots["CoM Velocity"].yLabel() = "[m/s]";

    ResizeAndFill3Dplot(n, [phases, phaseLength](size_t i, size_t j)
                        {return phases[i / phaseLength].controls()[i % phaseLength].acceleration(j);},
                        m_plots["CoM Acceleration (without gravity)"]);

    m_plots["CoM Velocity"].xLabel() = "t [s]";
    m_plots["CoM Velocity"].yLabel() = "[m/s^2]";

    ResizeAndFillVector(n, [phases, phaseLength](size_t i)
                        {return static_cast<double>(phases[i / phaseLength].controls()[i % phaseLength].left().multiplier());},
                        m_plots["u"]["Left"]);

    ResizeAndFillVector(n, [phases, phaseLength](size_t i)
                        {return static_cast<double>(phases[i / phaseLength].controls()[i % phaseLength].right().multiplier());},
                        m_plots["u"]["Right"]);

    m_plots["u"].xLabel() = "t [s]";
    m_plots["u"].yLabel() = "[1/s^2]";

    ResizeAndFill2Dplot(n, [phases, phaseLength](size_t i, size_t j)
                        {return phases[i / phaseLength].controls()[i % phaseLength].left().cop(j);},
                        m_plots["Left CoP"]);
    m_plots["Left CoP"].xLabel() = "t [s]";
    m_plots["Left CoP"].yLabel() = "[m]";

    ResizeAndFill2Dplot(n, [phases, phaseLength](size_t i, size_t j)
                        {return phases[i / phaseLength].controls()[i % phaseLength].right().cop(j);},
                        m_plots["Right CoP"]);
    m_plots["Right CoP"].xLabel() = "t [s]";
    m_plots["Right CoP"].yLabel() = "[m]";

}

void StepUpPlanner::Plotter::plot(const std::string &name, StepUpPlanner::Plot &plot)
{
    namespace plt = matplotlibcpp;

    plot.figureNumber() = plt::figure(plot.figureNumber());

    for (auto& element : plot.data()) {
        plt::named_plot(element.first, m_time, element.second);
    }

    plt::title(name);
    plt::legend();
    plt::xlabel(plot.xLabel());
    plt::ylabel(plot.yLabel());

    double yMin = plt::ylim()[0];
    double yMax = plt::ylim()[1];

    for (double& t : m_phaseTimings) {
        plt::plot({t, t}, {yMin, yMax}, "k--");
    }

    plt::ylim(yMin, yMax);

}

void StepUpPlanner::Plotter::plotAll(const std::vector<Phase> &phases)
{
    fillPlotsData(phases);

    for (auto& p : m_plots) {
        plot(p.first, p.second);
    }
}

StepUpPlanner::Plotter::Plotter()
{ }

StepUpPlanner::Plotter::~Plotter()
{ }

void StepUpPlanner::Plotter::plotFullSolutionBlocking(const std::vector<Phase> &phases)
{
    plotAll(phases);
    matplotlibcpp::show(true);

}

