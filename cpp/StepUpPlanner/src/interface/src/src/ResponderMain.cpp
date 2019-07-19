#include <Responder.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepUpPlanner::Responder>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
