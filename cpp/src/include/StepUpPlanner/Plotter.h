#ifndef STEPUPPLANNER_PLOTTER_H
#define STEPUPPLANNER_PLOTTER_H

#include <StepUpPlanner/Phase.h>
#include <vector>
#include <map>
#include <string>

namespace StepUpPlanner {
    class Plotter;
    class Plot;
}

class StepUpPlanner::Plot {
    std::map<std::string, std::vector<double>> m_data;
    long m_figureNumber;
    std::string m_xLabel, m_yLabel;

public:

    Plot();

    std::vector<double>& operator[](const std::string& key);

    std::map<std::string, std::vector<double>>& data();

    const std::map<std::string, std::vector<double>>& data() const;

    long& figureNumber();

    std::string& xLabel();

    std::string& yLabel();
};

class StepUpPlanner::Plotter {

    std::map<std::string, StepUpPlanner::Plot> m_plots;

    std::vector<double> m_time;
    std::vector<double> m_phaseTimings;

    size_t getNumberOfPoints(const std::vector<StepUpPlanner::Phase>& phases);

    void setTimeVector(const std::vector<StepUpPlanner::Phase>& phases);

    void fillPlotsData(const std::vector<StepUpPlanner::Phase>& phases);

    void plot(const std::string& name, Plot &plot);

    void plotAll(const std::vector<Phase> &phases);

public:

    Plotter();

    ~Plotter();

    void plotFullSolutionBlocking(const std::vector<StepUpPlanner::Phase>& phases);

};

#endif // STEPUPPLANNER_PLOTTER_H
