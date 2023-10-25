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

void mcp2515_select(void)
{}

void mcp2515_deselect()
{}

uint8_t mcp2515_spi_transfer(uint8_t data)
{
    return data;
}

void delay(uint32_t ms)
{
  for (; ms > 0; ms--)
  {
    for (uint16_t j = 0; j < 25000; j++)
    {
      __asm__("nop"); /* Do nothing. */
    }
  }
}

void sendCanData(uint32_t id, uint8_t dlc, uint8_t *data)  
{
    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = dlc;
    for (int i = 0; i < dlc; i++)
    {
        frame.data[i] = data[i];
    }
    
    mcp2515->sendMessage(&frame);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    mcp2515 = new MCP2515(&mcp2515_select,
                &mcp2515_deselect,
                &mcp2515_spi_transfer,
                &delay);

    uint8_t motorId = 1;
    mcp2515->reset();
    mcp2515->setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515->setNormalMode();

    TMotorActuators::AkActuators ak10 = TMotorActuators::AkActuators(motorId,
                                                            TMotorActuators::ak60_6_v1_1,
                                                            sendCanData);

    delay(1000);
    ak10.enable();
    delay(1000);

    ak10.move(0.0, -50, 0, 0.4, 0.2);
    delay(1000);

    ak10.move(3.1415, -50, -10, 2, 0.2);
    delay(1000);

    ak10.disable();
    delay(1000);
                
   
    // rclcpp::spin(std::make_shared<TestNode>());
    
    
    // rclcpp::shutdown();
    
    return 0;
}