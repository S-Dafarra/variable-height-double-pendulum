#include <rclcpp/rclcpp.hpp>
#include <Responder.h>
#include <controller_msgs/msg/center_of_mass_trajectory_message.hpp>
#include <controller_msgs/msg/footstep_data_list_message.hpp>
#include <controller_msgs/msg/step_up_planner_parameters_message.hpp>
#include <controller_msgs/msg/step_up_planner_request_message.hpp>
#include <controller_msgs/msg/step_up_planner_respond_message.hpp>
#include <controller_msgs/msg/step_up_planner_error_message.hpp>
#include <cassert>
#include <iostream>

#include <chrono>
using namespace std::chrono_literals;

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

class ErrorSubscriber : public rclcpp::Node
{
public:
    ErrorSubscriber()
        : Node("error_subscriber")
    {
        subscription_ = this->create_subscription<controller_msgs::msg::StepUpPlannerErrorMessage>(
            STEPUPPLANNER_ERRORS_TOPIC,
            [this](controller_msgs::msg::StepUpPlannerErrorMessage::UniquePtr errorMessage) {
                if (errorMessage->error_code) {
                    std::ostringstream asString;
                    asString << "Received error with code: " << std::to_string(errorMessage->error_code) << std::endl;
                    asString << "Error description: " << errorMessage->error_description << std::endl;
                    RCLCPP_INFO(this->get_logger(), asString.str());
                    ASSERT_IS_TRUE(false);
                } else {
                    index = errorMessage->sequence_id_received;
                    RCLCPP_INFO(this->get_logger(), "Received ack for message with ID: "
                                    + std::to_string(errorMessage->sequence_id_received));
                }
            });
    }
    ~ErrorSubscriber();

    unsigned int index = 0;
    unsigned char error_code = 0;

private:
    rclcpp::Subscription<controller_msgs::msg::StepUpPlannerErrorMessage>::SharedPtr subscription_;
};
ErrorSubscriber::~ErrorSubscriber(){}

class RespondSubscriber : public rclcpp::Node
{
public:
    RespondSubscriber()
        : Node("respond_subscriber")
    {
        m_subscription = this->create_subscription<controller_msgs::msg::StepUpPlannerRespondMessage>(
            STEPUPPLANNER_RESPOND_TOPIC,
            [this](controller_msgs::msg::StepUpPlannerRespondMessage::UniquePtr /*message*/) {
                std::ostringstream asString;
                asString << "Received message" << std::endl;

                messageReceived = true;

                RCLCPP_INFO(this->get_logger(), asString.str());
            });
    }
    ~RespondSubscriber();

    bool messageReceived = false;

private:
    rclcpp::Subscription<controller_msgs::msg::StepUpPlannerRespondMessage>::SharedPtr m_subscription;
};
RespondSubscriber::~RespondSubscriber(){}

class CoMSubscriber : public rclcpp::Node
{
public:
    CoMSubscriber(const std::string& topicName)
        : Node("com_subscriber")
    {
        m_subscription = this->create_subscription<controller_msgs::msg::CenterOfMassTrajectoryMessage>(
            topicName, [this](controller_msgs::msg::CenterOfMassTrajectoryMessage::UniquePtr message) {
                std::ostringstream asString;
                asString << "Received CoM message (ID " << message->euclidean_trajectory.queueing_properties.message_id <<")" << std::endl;

                messagesReceived++;

                RCLCPP_INFO(this->get_logger(), asString.str());
            });
    }
    ~CoMSubscriber();

    size_t messagesReceived = 0;

private:
    rclcpp::Subscription<controller_msgs::msg::CenterOfMassTrajectoryMessage>::SharedPtr m_subscription;
};
CoMSubscriber::~CoMSubscriber(){}

class FootstepsSubscriber : public rclcpp::Node
{
public:
    FootstepsSubscriber(const std::string& topicName)
        : Node("footsteps_subscriber")
    {
        m_subscription = this->create_subscription<controller_msgs::msg::FootstepDataListMessage>(
            topicName, [this](controller_msgs::msg::FootstepDataListMessage::UniquePtr message) {
                std::ostringstream asString;
                asString << "Received feet message: " << message->footstep_data_list.size() << " steps." << std::endl;

                messageReceived = true;

                RCLCPP_INFO(this->get_logger(), asString.str());
            });
    }
    ~FootstepsSubscriber();

    bool messageReceived = false;

private:
    rclcpp::Subscription<controller_msgs::msg::FootstepDataListMessage>::SharedPtr m_subscription;
};
FootstepsSubscriber::~FootstepsSubscriber(){}

controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr fillParametersMessage(unsigned int id,
                                                                                      const std::string& comMessageTopic,
                                                                                      const std::string& feetMessageTopic) {
    controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg =
        std::make_shared<controller_msgs::msg::StepUpPlannerParametersMessage>();

    std::vector<controller_msgs::msg::StepUpPlannerStepParameters> leftSteps;
    std::vector<controller_msgs::msg::StepUpPlannerStepParameters> rightSteps;

    leftSteps.resize(5);
    rightSteps.resize(5);

    for (size_t i = 0; i < leftSteps.size(); ++i) {
        leftSteps[i].foot_vertices.resize(4);
        leftSteps[i].foot_vertices[0].x = 0.05;
        leftSteps[i].foot_vertices[0].y = 0.05;
        leftSteps[i].foot_vertices[1].x = 0.05;
        leftSteps[i].foot_vertices[1].y = -0.05;
        leftSteps[i].foot_vertices[2].x = -0.05;
        leftSteps[i].foot_vertices[2].y = -0.05;
        leftSteps[i].foot_vertices[3].x = -0.05;
        leftSteps[i].foot_vertices[3].y = 0.05;

        rightSteps[i].set__foot_vertices(leftSteps[i].foot_vertices);
    }

    auto stand = leftSteps[0].STAND;
    auto swing = leftSteps[0].SWING;

    leftSteps[0].set__state(stand);
    rightSteps[0].set__state(stand);

    leftSteps[1].set__state(swing);
    rightSteps[1].set__state(stand);

    leftSteps[2].set__state(stand);
    rightSteps[2].set__state(stand);

    leftSteps[3].set__state(stand);
    rightSteps[3].set__state(swing);

    leftSteps[4].set__state(stand);
    rightSteps[4].set__state(stand);

    msg->phases_settings.resize(5);
    for (size_t p = 0; p < msg->phases_settings.size(); ++p) {
        msg->phases_settings[p].set__left_step_parameters(leftSteps[p]);
        msg->phases_settings[p].set__right_step_parameters(rightSteps[p]);
    }

    msg->phase_length = 30;
    msg->solver_verbosity = 1;
    msg->max_leg_length = 1.2;
    msg->ipopt_linear_solver = "ma27";
    msg->final_state_anticipation = 0.3;
    msg->static_friction_coefficient = 0.5;
    msg->torsional_friction_coefficient = 0.1;

    double N = msg->phase_length * msg->phases_settings.size();

    controller_msgs::msg::StepUpPlannerCostWeights weights;

    weights.cop = 10.0/N;
    weights.torques = 1.0/N;
    weights.control_multipliers = 0.1/N;
    weights.final_control = 1.0;
    weights.max_control_multiplier = 0.1;
    weights.final_state = 10.0;
    weights.control_variations = 1.0/N;
    weights.durations_difference = 5.0/msg->phases_settings.size();

    msg->set__cost_weights(weights);

    msg->sequence_id = id;

    msg->send_com_messages = true;
    msg->com_messages_topic = comMessageTopic;
    msg->max_com_message_length = 50;
    msg->include_com_messages = true;

    msg->send_footstep_messages = true;
    msg->footstep_messages_topic = feetMessageTopic;
    msg->include_footstep_messages = true;

    return msg;
}

controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr fillRequestMessage() {

    controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg =
        std::make_shared<controller_msgs::msg::StepUpPlannerRequestMessage>();

    msg->initial_com_position.x = 0.0;
    msg->initial_com_position.y = 0.0;
    msg->initial_com_position.z = 1.16;

    msg->initial_com_velocity.x = 0.0;
    msg->initial_com_velocity.y = 0.0;
    msg->initial_com_velocity.z = 0.0;

    msg->desired_com_position.x = 0.6;
    msg->desired_com_position.y = 0.0;
    msg->desired_com_position.z = 1.56;

    msg->desired_com_velocity.x = 0.0;
    msg->desired_com_velocity.y = 0.0;
    msg->desired_com_velocity.z = 0.0;

    msg->phases.resize(5);
    geometry_msgs::msg::Quaternion identityQuaternion;
    identityQuaternion.w = 1.0;
    identityQuaternion.x = 0.0;
    identityQuaternion.y = 0.0;
    identityQuaternion.z = 0.0;

    msg->phases[0].left_foot_pose.position.x = 0.0;
    msg->phases[0].left_foot_pose.position.y = 0.15;
    msg->phases[0].left_foot_pose.position.z = 0.0;
    msg->phases[0].left_foot_pose.set__orientation(identityQuaternion);

    msg->phases[0].right_foot_pose.position.x = 0.0;
    msg->phases[0].right_foot_pose.position.y = -0.15;
    msg->phases[0].right_foot_pose.position.z = 0.0;
    msg->phases[0].right_foot_pose.set__orientation(identityQuaternion);

    msg->phases[1].set__right_foot_pose(msg->phases[0].right_foot_pose);

    msg->phases[2].left_foot_pose.position.x = 0.6;
    msg->phases[2].left_foot_pose.position.y = 0.15;
    msg->phases[2].left_foot_pose.position.z = 0.4;
    msg->phases[2].left_foot_pose.set__orientation(identityQuaternion);
    msg->phases[2].set__right_foot_pose(msg->phases[0].right_foot_pose);

    msg->phases[3].set__left_foot_pose(msg->phases[2].left_foot_pose);

    msg->phases[4].set__left_foot_pose(msg->phases[2].left_foot_pose);
    msg->phases[4].right_foot_pose.position.x =   0.6;
    msg->phases[4].right_foot_pose.position.y = -0.15;
    msg->phases[4].right_foot_pose.position.z =   0.4;
    msg->phases[4].right_foot_pose.set__orientation(identityQuaternion);


    msg->desired_leg_length = 1.18;
    msg->left_desired_final_control.cop.x = 0.0;
    msg->left_desired_final_control.cop.y = 0.0;
    msg->right_desired_final_control.set__cop(msg->left_desired_final_control.cop);

    msg->phases[0].minimum_duration = 0.5;
    msg->phases[0].maximum_duration = 2.0;
    msg->phases[0].desired_duration = 0.8;

    msg->phases[1].minimum_duration = 0.5;
    msg->phases[1].maximum_duration = 2.0;
    msg->phases[1].desired_duration = 1.2;

    msg->phases[2].minimum_duration = 0.5;
    msg->phases[2].maximum_duration = 2.0;
    msg->phases[2].desired_duration = 0.8;

    msg->phases[3].minimum_duration = 0.5;
    msg->phases[3].maximum_duration = 2.0;
    msg->phases[3].desired_duration = 1.2;

    msg->phases[4].minimum_duration = 0.5;
    msg->phases[4].maximum_duration = 2.0;
    msg->phases[4].desired_duration = 0.8;

    double desiredLeftMultiplier = static_cast<double>(9.81/(2.0*(msg->desired_com_position.z -
                                                                      msg->phases[4].left_foot_pose.position.z)));

    msg->left_desired_final_control.set__multiplier(desiredLeftMultiplier);

    double desiredRightMultiplier = static_cast<double>(9.81/(2.0*(msg->desired_com_position.z -
                                                                       msg->phases[4].right_foot_pose.position.z)));

    msg->right_desired_final_control.set__multiplier(desiredRightMultiplier);

    return msg;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(500ms);

    auto responderPublisher = std::make_shared<StepUpPlanner::Responder>();
    auto errorSubscriber = std::make_shared<ErrorSubscriber>();
    auto respondSubscriber = std::make_shared<RespondSubscriber>();

    std::string comTopic = "/us/ihmc/CoMtest";
    std::string feetTopic = "/us/ihmc/FeetTest";

    auto comSubscriber = std::make_shared<CoMSubscriber>(comTopic);
    auto footStepsSubscriber = std::make_shared<FootstepsSubscriber>(feetTopic);

    auto parametersPublisherNode = rclcpp::Node::make_shared("parameters_publisher");
    auto parametersPublisher = parametersPublisherNode->create_publisher<controller_msgs::msg::StepUpPlannerParametersMessage>
                               (STEPUPPLANNER_PARAMETERS_TOPIC);

    auto requestPublisherNode = rclcpp::Node::make_shared("request_publisher");
    auto requestPublisher = parametersPublisherNode->create_publisher<controller_msgs::msg::StepUpPlannerRequestMessage>
                            (STEPUPPLANNER_REQUEST_TOPIC);

    auto parametersMessage = fillParametersMessage(1, comTopic, feetTopic);
    loop_rate.sleep(); //This sleep is necessary to make sure that the nodes are registered
    parametersPublisher->publish(parametersMessage);

    int i = 0;
    while (!(errorSubscriber->index) && i < 10) {
        std::cout << "Loop" << std::endl;
        rclcpp::spin_some(parametersPublisherNode);
        rclcpp::spin_some(responderPublisher);
        rclcpp::spin_some(errorSubscriber);

        loop_rate.sleep();
        ++i;

        if (i > 4) {
            std::cout << "Sending again..." << std::endl;
            parametersPublisher->publish(parametersMessage);
        }
    }

    auto requestMessage = fillRequestMessage();
    requestPublisher->publish(requestMessage);
    std::cout << "Request sent..." << std::endl;
    i = 0;
    respondSubscriber->messageReceived = false;

    while (!((respondSubscriber->messageReceived) && (comSubscriber->messagesReceived == 5) && (footStepsSubscriber->messageReceived))
           && (i < 100)) {
        std::cout << "Loop " << i << std::endl;
        rclcpp::spin_some(requestPublisherNode);
        rclcpp::spin_some(responderPublisher);
        rclcpp::spin_some(errorSubscriber);
        rclcpp::spin_some(respondSubscriber);
        rclcpp::spin_some(comSubscriber);
        rclcpp::spin_some(footStepsSubscriber);
        ++i;
    }
    ASSERT_IS_TRUE(i != 100);

    rclcpp::shutdown();
    errorSubscriber = nullptr;
    return EXIT_SUCCESS;
}
