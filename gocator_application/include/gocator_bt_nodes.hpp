#pragma once

#include <atomic>

// behavior tree common node headers import
// pulls in all the common behavior tree nodes for ROS2
#include <bt_common_ros_headers.hpp>
#include <gocator_driver/gocator_commands.h>


namespace gocator_bt_nodes 
{
  /**
   * Your custom behavior tree nodes can be defined here.
   * Remember to register them with the factory in the GocatorWidget class in gocator_widget.cpp
   * 
   */

   /**
      // Example of a custom synchronous action node
      class ExampleBTNode : public BT::SyncActionNode
      {
          public:
            inline static std::string ExampleInputKey = "example_input";
            inline static std::string ExampleOutputKey = "example_output";
            
            // Constructor
            ExampleBTNode(const std::string& name, const BT::NodeConfiguration& config)
              : BT::SyncActionNode(name, config) 
            {
              // Initialization code here
            }

            // This method is used to define the ports of the node
            static BT::PortsList providedPorts()
            {
              return{ BT::InputPort<std::string>(ExampleInputKey),
                      BT::OutputPort<std::string>(ExampleOutputKey) };
            }

            // This is the main method that is called when the node is ticked
            BT::NodeStatus tick() override
            {
              // Node execution code here
              return BT::NodeStatus::SUCCESS;
            }
        };
    */
class InitializeScanningSystem : public BT::SyncActionNode
  {
    public:
      inline static std::string IP_ADDRESS_INPUT_KEY = "ip_address";
      inline static std::string GOCATOR_COMMANDER_OUTPUT = "gocator_commander_object";
      
      // Constructor
      InitializeScanningSystem(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) 
      {
      }

      // This method is used to define the ports of the node
      static BT::PortsList providedPorts()
      {
        return{ BT::InputPort<std::string>(IP_ADDRESS_INPUT_KEY),
                BT::OutputPort<GocatorCommander*>(GOCATOR_COMMANDER_OUTPUT) };
      }

      // This is the main method that is called when the node is ticked
      BT::NodeStatus tick() override
      {
        std::string ip = getBTInput<std::string>(this, IP_ADDRESS_INPUT_KEY);
        auto commander = new GocatorCommander(ip);
        setOutput(GOCATOR_COMMANDER_OUTPUT, commander);
        if (commander->system_status == kOK) 
        {
          std::cout << "Scanning System Initialized!" << std::endl;
          return BT::NodeStatus::SUCCESS;
        }
        else return BT::NodeStatus::FAILURE;
      }
  };

  class TriggerScan : public BT::SyncActionNode
  {
    public:
      inline static std::string DIST_INPUT_KEY = "scan_distance";
      inline static std::string ROBOT_SPEED_INPUT_KEY = "robot_velocity";
      inline static std::string EXPOSURE_INPUT_KEY = "scanner_exposure_value";
      inline static std::string OUTPUT_DIR_INPUT_KEY = "output_directory";

      inline static std::string GOCATOR_COMMANDER_INPUT = "gocator_commander_object";
      
      // Constructor
      TriggerScan(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) 
      {
      }

      // This method is used to define the ports of the node
      static BT::PortsList providedPorts()
      {
        return{ BT::InputPort<float>(DIST_INPUT_KEY),
                BT::InputPort<float>(ROBOT_SPEED_INPUT_KEY),
                BT::InputPort<float>(EXPOSURE_INPUT_KEY),
                BT::InputPort<std::string>(OUTPUT_DIR_INPUT_KEY),
                BT::InputPort<GocatorCommander*>(GOCATOR_COMMANDER_INPUT) };
      }

      // This is the main method that is called when the node is ticked
      BT::NodeStatus tick() override
      {
        float dist = getBTInput<float>(this, DIST_INPUT_KEY);
        float exposure = getBTInput<float>(this, EXPOSURE_INPUT_KEY);
        float speed = getBTInput<float>(this, ROBOT_SPEED_INPUT_KEY);
        auto commander = getBTInput<GocatorCommander*>(this, GOCATOR_COMMANDER_INPUT);

        std::string output_dir = getBTInput<std::string>(this, OUTPUT_DIR_INPUT_KEY);
        commander->setRobotSpeed(speed);
        commander->setScanDistance(dist);
        commander->setExposure(exposure);
        int c_idx = this->config().blackboard->get<int>("cloud_count");
        this->config().blackboard->set("cloud_count", c_idx+1);
        auto status = commander->performScan(output_dir, c_idx);
        if (status == kOK) {
          std::cout << "Scan Complete!" << std::endl;
          return BT::NodeStatus::SUCCESS;
        }
        else return BT::NodeStatus::FAILURE;
      }
  };

}
