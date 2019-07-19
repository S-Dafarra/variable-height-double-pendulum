#include <Responder.h>
#include <cassert>

using std::placeholders::_1;

void StepUpPlanner::Responder::respond(const controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg)
{
    if (!m_solver.isReady()) {
        sendErrorMessage(Errors::PARAMETERS_NOT_SET,
                         "Parameters were not specified.");
        return;
    }

    if (msg->phases.size() != m_solver.numberOfPhases()) {
        sendErrorMessage(Errors::REQUEST_ERROR,
                         "The number of phases is different from the one specified in the parameters message");
        return;
    }

    for (size_t p = 0; p < m_solver.numberOfPhases(); ++p) {
        StepUpPlanner::Phase& phase = m_solver.getPhase(p);
        if ((phase.getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT) ||
            (phase.getPhaseType() == StepUpPlanner::PhaseType::DOUBLE_SUPPORT)) {
            auto& leftPosition = msg->phases[p].left_foot_pose.position;
            phase.setLeftPosition(leftPosition.x, leftPosition.y, leftPosition.z);
            auto& leftQuaternion = msg->phases[p].left_foot_pose.orientation;
            phase.leftRotation().setFromQuaternion(leftQuaternion.w, leftQuaternion.x, leftQuaternion.y, leftQuaternion.z);
        }

        if ((phase.getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT) ||
            (phase.getPhaseType() == StepUpPlanner::PhaseType::DOUBLE_SUPPORT)) {
            auto& rightPosition = msg->phases[p].right_foot_pose.position;
            phase.setRightPosition(rightPosition.x, rightPosition.y, rightPosition.z);
            auto& rightQuaternion = msg->phases[p].right_foot_pose.orientation;
            phase.rightRotation().setFromQuaternion(rightQuaternion.w, rightQuaternion.x, rightQuaternion.y, rightQuaternion.z);
        }

        phase.setDurationSettings(msg->phases[p].minimum_duration, msg->phases[p].maximum_duration, msg->phases[p].desired_duration);
    }

    StepUpPlanner::State initialState;
    initialState.setPosition(msg->initial_com_position.x, msg->initial_com_position.y, msg->initial_com_position.z);
    initialState.setVelocity(msg->initial_com_velocity.x, msg->initial_com_velocity.y, msg->initial_com_velocity.z);

    StepUpPlanner::References references;
    references.zero();
    references.desiredState().setPosition(msg->desired_com_position.x, msg->desired_com_position.y, msg->desired_com_position.z);
    references.desiredState().setVelocity(msg->desired_com_velocity.x, msg->desired_com_velocity.y, msg->desired_com_velocity.z);

    references.desiredControl().left().setMultiplier(msg->left_desired_final_control.multiplier);
    references.desiredControl().left().setCoP(msg->left_desired_final_control.cop.x, msg->left_desired_final_control.cop.y);
    references.desiredControl().right().setMultiplier(msg->right_desired_final_control.multiplier);
    references.desiredControl().right().setCoP(msg->right_desired_final_control.cop.x, msg->right_desired_final_control.cop.y);

    bool ok = references.setDesiredLegLength(msg->desired_leg_length);
    if (!ok) {
        sendErrorMessage(Errors::REQUEST_ERROR, "The desired leg length is not supposed to be smaller than 0.");
        return;
    }

    ok = m_solver.solve(initialState, references);
    if (!ok) {
        sendErrorMessage(Errors::SOLVER_ERROR, "Failed to solve the optimization problem.");
        return;
    }

    sendRespondMessage();
}

void StepUpPlanner::Responder::processParameters(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "[processParameters] Parameters received");

    m_phases.resize(msg->phases_settings.size());

    for (size_t phase = 0; phase < m_phases.size(); ++phase) {
        StepUpPlanner::Step leftStep, *leftPointer, rightStep, *rightPointer;

        auto& setting = msg->phases_settings[phase];
        auto& leftParameters = setting.left_step_parameters;
        auto& rightParameters = setting.right_step_parameters;

        if (leftParameters.state == leftParameters.STAND) {

            leftPointer = &leftStep;

            std::vector<StepUpPlanner::Vertex> vertices;
            vertices.resize(leftParameters.foot_vertices.size());
            for (size_t vertex = 0; vertex < leftParameters.foot_vertices.size(); ++vertex) {
                vertices[vertex].x = leftParameters.foot_vertices[vertex].x;
                vertices[vertex].y = leftParameters.foot_vertices[vertex].y;
            }

            bool ok = leftStep.setVertices(vertices);

            if (!ok) {
                sendErrorMessage(Errors::PARAMETERS_ERROR,
                                 "Failed to set the vertices for the left foot in the phase with index" + std::to_string(phase));
                return;
            }

        } else {
            leftPointer = nullptr;
        }

        if (setting.right_step_parameters.state == setting.right_step_parameters.STAND) {
            rightPointer = &rightStep;

            std::vector<StepUpPlanner::Vertex> vertices;
            vertices.resize(rightParameters.foot_vertices.size());
            for (size_t vertex = 0; vertex < rightParameters.foot_vertices.size(); ++vertex) {
                vertices[vertex].x = rightParameters.foot_vertices[vertex].x;
                vertices[vertex].y = rightParameters.foot_vertices[vertex].y;
            }

            bool ok = rightStep.setVertices(vertices);

            if (!ok) {
                sendErrorMessage(Errors::PARAMETERS_ERROR,
                                 "Failed to set the vertices for the right foot in the phase with index" + std::to_string(phase));
                return;
            }

        } else {
            rightPointer = nullptr;
        }

        m_phases[phase] = StepUpPlanner::Phase(leftPointer, rightPointer);
    }

    StepUpPlanner::Settings settings;

    settings.phaseLength() = msg->phase_length;
    settings.solverVerbosity() = msg->solver_verbosity;

    bool ok = settings.setMaximumLegLength(msg->max_leg_length);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The max_leg_length is supposed to be a positive number");
        return;
    }

    ok = settings.setIpoptLinearSolver(msg->ipopt_linear_solver);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "Unknown solver name. "
                         "See options at https://www.coin-or.org/Ipopt/documentation/node51.html#SECTION0001111010000000000000.");
        return;
    }

    ok = settings.setFinalStateAnticipation(msg->final_state_anticipation);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The final state anticipation is supposed to be in the interval [0.0, 1.0].");
        return;
    }

    settings.setStaticFrictionCoefficient(msg->static_friction_coefficient);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The staticFrictionCoeff is supposed to be a positive number.");
        return;
    }

    settings.setTorsionalFrictionCoefficient(msg->torsional_friction_coefficient);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The torsionalFrictionCoeff is supposed to be a positive number.");
        return;
    }

    StepUpPlanner::CostWeights& weights = settings.costWeights();
    auto& messageWeights = msg->cost_weights;
    weights.cop = messageWeights.cop;
    weights.torques = messageWeights.torques;
    weights.multipliers = messageWeights.control_multipliers;
    weights.finalControl = messageWeights.final_control;
    weights.maxMultiplier = messageWeights.max_control_multiplier;
    weights.finalStateError = messageWeights.final_state;
    weights.controlVariations = messageWeights.control_variations;
    weights.durationsDifference = messageWeights.durations_difference;

    ok = m_solver.resetProblem(m_phases, settings);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "Failed to reset the problem.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[processParameters] Parameters set correctly.");

    ackReceivedParameters(msg->sequence_id);

}

void StepUpPlanner::Responder::sendErrorMessage(StepUpPlanner::Responder::Errors errorType, const std::string &errorMessage)
{
    auto error = std::make_shared<controller_msgs::msg::StepUpPlannerErrorMessage>();

    if (errorType == Errors::PARAMETERS_ERROR) {
        error->error_code = 1;
    } else if (errorType == Errors::PARAMETERS_NOT_SET) {
        error->error_code = 2;
    } else if (errorType == Errors::REQUEST_ERROR) {
        error->error_code = 3;
    } else if (errorType == Errors::SOLVER_ERROR) {
        error->error_code = 4;
    }

    error->error_description = errorMessage;

    m_errorPublisher->publish(error);

    RCLCPP_ERROR(this->get_logger(), "[StepUpPlanner::Responder] " + errorMessage);
}

void StepUpPlanner::Responder::sendRespondMessage()
{
    bool ok = m_solver.getFullSolution(m_phases);
    assert(ok);

    m_respondMessage->phases_result.resize(m_phases.size());

    for (size_t p = 0; p < m_phases.size(); ++p) {
        auto& phaseResult = m_respondMessage->phases_result[p];

        phaseResult.duration = static_cast<double>(m_phases[p].duration());

        phaseResult.com_position.resize(m_phases[p].states().size());
        phaseResult.com_velocity.resize(m_phases[p].states().size());

        for (size_t s = 0; s < m_phases[p].states().size(); ++s) {
            phaseResult.com_position[s].x = m_phases[p].states()[s].position(0);
            phaseResult.com_position[s].y = m_phases[p].states()[s].position(1);
            phaseResult.com_position[s].z = m_phases[p].states()[s].position(2);

            phaseResult.com_velocity[s].x = m_phases[p].states()[s].velocity(0);
            phaseResult.com_velocity[s].y = m_phases[p].states()[s].velocity(1);
            phaseResult.com_velocity[s].z = m_phases[p].states()[s].velocity(2);
        }

        phaseResult.com_acceleration.resize(m_phases[p].controls().size());
        phaseResult.left_controls.resize(m_phases[p].controls().size());
        phaseResult.right_controls.resize(m_phases[p].controls().size());

        for (size_t c = 0; c < m_phases[p].controls().size(); ++c) {

            phaseResult.com_acceleration[c].x = m_phases[p].controls()[c].acceleration(0);
            phaseResult.com_acceleration[c].y = m_phases[p].controls()[c].acceleration(1);
            phaseResult.com_acceleration[c].z = m_phases[p].controls()[c].acceleration(2);

            phaseResult.left_controls[c].multiplier = static_cast<double>(m_phases[p].controls()[c].left().multiplier());
            phaseResult.left_controls[c].cop.x = m_phases[p].controls()[c].left().cop(0);
            phaseResult.left_controls[c].cop.y = m_phases[p].controls()[c].left().cop(1);

            phaseResult.right_controls[c].multiplier = static_cast<double>(m_phases[p].controls()[c].right().multiplier());
            phaseResult.right_controls[c].cop.x = m_phases[p].controls()[c].right().cop(0);
            phaseResult.right_controls[c].cop.y = m_phases[p].controls()[c].right().cop(1);
        }
    }

    m_respondPublisher->publish(m_respondMessage);

}

void StepUpPlanner::Responder::ackReceivedParameters(unsigned int message_id)
{
    auto error = std::make_shared<controller_msgs::msg::StepUpPlannerErrorMessage>();
    error->error_description = "";
    error->error_code = 0;
    error->sequence_id_received = message_id;
    m_errorPublisher->publish(error);
}

StepUpPlanner::Responder::Responder()
: Node("StepUpPlannerResponder")
{
    m_respondMessage = std::make_shared<controller_msgs::msg::StepUpPlannerRespondMessage>();
    m_respondPublisher = this->create_publisher<controller_msgs::msg::StepUpPlannerRespondMessage>(STEPUPPLANNER_RESPOND_TOPIC);
    m_errorPublisher = this->create_publisher<controller_msgs::msg::StepUpPlannerErrorMessage>(STEPUPPLANNER_ERRORS_TOPIC);
    m_requestSubscriber = this->create_subscription<controller_msgs::msg::StepUpPlannerRequestMessage>(
        STEPUPPLANNER_REQUEST_TOPIC, std::bind(&StepUpPlanner::Responder::respond, this, _1));
    m_parametersSubscriber = this->create_subscription<controller_msgs::msg::StepUpPlannerParametersMessage>(
        STEPUPPLANNER_PARAMETERS_TOPIC, std::bind(&StepUpPlanner::Responder::processParameters, this, _1));
}

