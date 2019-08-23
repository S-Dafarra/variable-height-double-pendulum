#include <Responder.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto responder = std::make_shared<StepUpPlanner::Responder>();
    RCLCPP_INFO(responder->get_logger(), "Running...");
    rclcpp::spin(responder);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
