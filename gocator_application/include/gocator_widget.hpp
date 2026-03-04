#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/node.hpp"
#include "QWidget"


namespace Ui {
  class GocatorLineScanner;
}
namespace gocator_widget 
{

class GocatorWidget : public QWidget
{

  Q_OBJECT

  public:
    explicit GocatorWidget(rclcpp::Node::SharedPtr ros_node, QWidget *parent = nullptr);

    ~GocatorWidget();

  private:
    rclcpp::Node::SharedPtr ros_node_;
    Ui::GocatorLineScanner* ui_;
    BT::Tree tree;
    BT::Blackboard::Ptr blackboard;
};

} // namespace gocator_widget