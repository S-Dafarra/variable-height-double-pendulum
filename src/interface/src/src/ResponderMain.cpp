#include <Responder.h>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto responder = std::make_shared<StepUpPlanner::Responder>();
    RCLCPP_INFO(responder->get_logger(), "Running...");

    rclcpp::WallRate loop_rate(1ms);

    size_t i = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(responder);
        ++i;
        if (i == 10) {
            responder->drawFigures();
            i = 0;
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
