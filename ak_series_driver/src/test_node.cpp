#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
// #include <include/linux/can.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ak_series_driver/mit_params.hpp"
#include "ak_series_driver/tmotor_ak_actuators.hpp"
#include "ak_series_driver/mcp2515.h"
#include "ak_series_driver/can.h"
#include "ak_series_driver/mcp2515_speed_cfg.h"

#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;
MCP2515 *mcp2515;
class TestNode : public rclcpp::Node
{

    
    public:


        TestNode()
        : Node("driver")
        { }

};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
  
                
   
    // rclcpp::spin(std::make_shared<TestNode>());
    
    
    // rclcpp::shutdown();
    
    return 0;
}