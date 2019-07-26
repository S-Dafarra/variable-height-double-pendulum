#ifndef STEPUPPLANNER_IF_RESPONDER_H
#define STEPUPPLANNER_IF_RESPONDER_H

#include <rclcpp/rclcpp.hpp>
#include <StepUpPlanner/Solver.h>
#include <StepUpPlanner/Plotter.h>
#include <controller_msgs/msg/step_up_planner_parameters_message.hpp>
#include <controller_msgs/msg/step_up_planner_request_message.hpp>
#include <controller_msgs/msg/step_up_planner_respond_message.hpp>
#include <controller_msgs/msg/step_up_planner_error_message.hpp>
#include <controller_msgs/msg/center_of_mass_trajectory_message.hpp>
#include <controller_msgs/msg/pelvis_height_trajectory_message.hpp>
#include <controller_msgs/msg/footstep_data_list_message.hpp>
#include <vector>
#include <string>

#define STEPUPPLANNER_REQUEST_TOPIC    "/us/ihmc/stepUpPlanner/request"
#define STEPUPPLANNER_RESPOND_TOPIC    "/us/ihmc/stepUpPlanner/respond"
#define STEPUPPLANNER_PARAMETERS_TOPIC "/us/ihmc/stepUpPlanner/parameters"
#define STEPUPPLANNER_ERRORS_TOPIC     "/us/ihmc/stepUpPlanner/errors"

namespace StepUpPlanner {
    class Responder;
}

class StepUpPlanner::Responder : public rclcpp::Node {

    enum class Errors {
        PARAMETERS_ERROR,
        PARAMETERS_NOT_SET,
        REQUEST_ERROR,
        SOLVER_ERROR
    };

    rclcpp::Publisher<controller_msgs::msg::StepUpPlannerRespondMessage>::SharedPtr m_respondPublisher;
    rclcpp::Publisher<controller_msgs::msg::StepUpPlannerErrorMessage>::SharedPtr m_errorPublisher;
    rclcpp::Subscription<controller_msgs::msg::StepUpPlannerParametersMessage>::SharedPtr m_parametersSubscriber;
    rclcpp::Subscription<controller_msgs::msg::StepUpPlannerRequestMessage>::SharedPtr m_requestSubscriber;
    controller_msgs::msg::StepUpPlannerRespondMessage::SharedPtr m_respondMessage;
    rclcpp::Publisher<controller_msgs::msg::CenterOfMassTrajectoryMessage>::SharedPtr m_CoMMessagePublisher;
    rclcpp::Publisher<controller_msgs::msg::PelvisHeightTrajectoryMessage>::SharedPtr m_pelvisMessagePublisher;
    rclcpp::Publisher<controller_msgs::msg::FootstepDataListMessage>::SharedPtr m_feetMessagePublisher;
    std::vector<controller_msgs::msg::CenterOfMassTrajectoryMessage::SharedPtr> m_CoMMessages;
    size_t m_CoMMessagesPerPhase;
    std::vector<controller_msgs::msg::PelvisHeightTrajectoryMessage::SharedPtr> m_pelvisMessages;
    size_t m_pelvisMessagesPerPhase;
    double m_pelvisHeightDelta;
    controller_msgs::msg::FootstepDataListMessage::SharedPtr m_feetMessage;

    std::vector<StepUpPlanner::Phase> m_phases;
    StepUpPlanner::Solver m_solver;
    StepUpPlanner::Plotter m_plotter;

    bool prepareSolver(const controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg, State &initialState, References &references);

    void respond(const controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg);

    bool processPhaseSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg);

    bool processCoMMessagesSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg, const Settings &settings);

    bool processPelvisHeightMessagesSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg,
                                             const Settings &settings);

    bool processFootStepMessagesSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg);

    bool processPlannerSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg, Settings &settings);

    void processParameters(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg);

    void sendErrorMessage(Errors errorType, const std::string& errorMessage);

    void prepareRespondMessage();

    void sendCenterOfMassTrajectoryMessages();

    void sendPelvisHeightTrajectoryMessages();

    void sendFootStepDataListMessage();

    void ackReceivedParameters(unsigned int message_id);

public:

    Responder();

    void drawFigures();

};




#endif // STEPUPPLANNER_IF_RESPONDER_H
