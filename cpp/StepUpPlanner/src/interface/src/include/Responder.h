#ifndef STEPUPPLANNER_IF_RESPONDER_H
#define STEPUPPLANNER_IF_RESPONDER_H

#include <rclcpp/rclcpp.hpp>
#include <StepUpPlanner/Solver.h>
#include <controller_msgs/msg/step_up_planner_parameters_message.hpp>
#include <controller_msgs/msg/step_up_planner_request_message.hpp>
#include <controller_msgs/msg/step_up_planner_respond_message.hpp>
#include <controller_msgs/msg/step_up_planner_error_message.hpp>
#include <vector>
#include <string>

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
    controller_msgs::msg::StepUpPlannerRespondMessage m_respondMessage;
    std::vector<StepUpPlanner::Phase> m_phases;
    StepUpPlanner::Solver m_solver;

    void respond(const controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg);

    void processParameters(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg);

    void sendErrorMessage(Errors errorType, const std::string& errorMessage);

    void sendRespondMessage();

public:

    Responder();

};




#endif // STEPUPPLANNER_IF_RESPONDER_H
