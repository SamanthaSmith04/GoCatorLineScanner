#include "gocator_widget.hpp"
#include "QApplication"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // Create ROS2 Node
    auto node = std::make_shared<rclcpp::Node>("gocator_widget_node");

    // Start GUI application
    gocator_widget::GocatorWidget form(node, nullptr);
    form.show();
    auto ret = app.exec();

    rclcpp::shutdown();
    return ret;
}
