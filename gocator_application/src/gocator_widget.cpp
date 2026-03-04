#include "gocator_widget.hpp"

// behavior tree common node headers
#include <action_nodes.hpp>
#include <condition_nodes.hpp>
#include <bt_ros_nodes.hpp>

// Your custom behavior tree nodes
#include <gocator_bt_nodes.hpp>

#include <QTimer>
#include <filesystem>
#include <chrono>
#include <iostream>

#include "ui_gocator.h" // will always be "ui_" + <ui file name> + ".h"

namespace gocator_widget
{

GocatorWidget::GocatorWidget(rclcpp::Node::SharedPtr ros_node, QWidget *parent)
    : QWidget(parent), ros_node_(ros_node), ui_(new Ui::GocatorLineScanner)
{

  // find the file for the behavior tree configuration
  std::string package_path = ament_index_cpp::get_package_share_directory("gocator_application");
  std::string gui_bt_file = package_path + "/config/gocator_bt.xml";  
 
  // Set up the QT UI components
  ui_->setupUi(this);

  // set up session folder
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);

  std::ostringstream oss;
  oss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
  std::string timestamp = oss.str();
  std::string save_path_files = std::string(getenv("HOME")) + "/.aims/aidf/" + timestamp;

  // create folder if it doesnt already exist
  if(!std::filesystem::exists(save_path_files)) {
    std::filesystem::create_directories(save_path_files);
  }

  // initialize the UI for the GUI
  blackboard = BT::Blackboard::create();

  // SET Blackboard values here
  blackboard->set("main_gui", static_cast<QWidget*>(this));
  blackboard->set("trigger_scan_push_button", static_cast<QAbstractButton*>(ui_->trigger_scan_push_button));
  blackboard->set("scan_distance_double_spin_box", static_cast<QAbstractSpinBox*>(ui_->scan_distance_double_spin_box));
  blackboard->set("scanner_exposure_double_spin_box", static_cast<QAbstractSpinBox*>(ui_->scanner_exposure_double_spin_box));
  blackboard->set("robot_speed_double_spin_box", static_cast<QAbstractSpinBox*>(ui_->robot_speed_double_spin_box));
  blackboard->set("init_system_push_button", static_cast<QAbstractButton*>(ui_->init_system_push_button));
  blackboard->set("ip_address_line_edit", static_cast<QLineEdit*>(ui_->ip_address_line_edit));

  blackboard->set("project_folder_path", save_path_files);
  blackboard->set("cloud_count", 0);
  
  // Register custom nodes in the factory if any
  BT::BehaviorTreeFactory factory;
  
  // Do any other GUI / BT setup here
  factory.registerNodeType<bt_common_nodes::WaitForGuiInput>("WaitForGuiInput");
  factory.registerNodeType<bt_common_nodes::GetValueFromLineEdit>("GetValueFromLineEdit");
  factory.registerNodeType<bt_common_nodes::GetValueFromSpinBox>("GetValueFromSpinBox");
  factory.registerNodeType<gocator_bt_nodes::InitializeScanningSystem>("InitializeScanningSystem");
  factory.registerNodeType<gocator_bt_nodes::TriggerScan>("TriggerScan");
  
  // Create the Behavior Tree
  this->tree = factory.createTreeFromFile(gui_bt_file, blackboard);
  

  // Start ticking Behavior Tree
  QTimer *tree_timer = new QTimer(this);
  QObject::connect(tree_timer, &QTimer::timeout, [this]() {
      BT::NodeStatus status = this->tree.tickOnce();
  });
  tree_timer->start(10); // Tick every 10 ms

}

GocatorWidget::~GocatorWidget()
{
  delete ui_;
}

} // namespace gocator_widget