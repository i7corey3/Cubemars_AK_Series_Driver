#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <thread>
#include <chrono>

#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ak_series_driver/mit_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ak_series_driver/can_communication.hpp"
#include "ak_series_driver/ak_driver.hpp"



#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U


using std::placeholders::_1;
using namespace std::chrono_literals;


unsigned char find_can(const int port)
{
	char buf[128]={0};
	sprintf(buf,"/sys/class/net/can%d/can_bittiming/bitrate",port);
	return ((access(buf,0)==0));
}


class SingleMotor : public rclcpp::Node
{

    
    public:

        SingleMotor()
        : Node("driver")
        { 
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "/ak_driver/motor_control", 10, std::bind(&SingleMotor::topic_callback, this, _1));

            
            publisher_ = this->create_publisher<std_msgs::msg::String>("/ak_driver/motor_status", 10);
            //std::thread status(&SingleMotor::get_status, this);
           setup();
        }

        void setup()
        {
            driver.setup(0, 1000000);
            printf("Setup Complete\r\n");
            
            
        }

 

    private:
        AKDriver driver;
        
        std::vector<float> data = {0.0, 0.0, 0.0, 0.0, 0.0};
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        uint8_t addr[2] = {0x68, 0x6A};
        
        
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       
       
        void get_status()
        {
            
            auto message = std_msgs::msg::String();
            //std::vector<uint8_t> can = driver.get_can_frame();
            std::stringstream ss; 

            //for (auto it = can.begin(); it != can.end(); it++) { 
            //    ss << *it << " "; 
            //} 
            message.data = ss.str();
            
            publisher_->publish(message);
        }

        void topic_callback(const std_msgs::msg::String::SharedPtr msg) 
        {
            
            std::string cmd;
            std::vector<std::string> inputs;
            cmd = msg->data.c_str();
            size_t pos = cmd.find(' ');
            size_t initialPos = 0;
            while( pos != std::string::npos)
            {
                inputs.push_back( cmd.substr( initialPos, pos - initialPos));
                initialPos = pos + 1;
                pos = cmd.find(' ', initialPos);
            }
            inputs.push_back( cmd.substr( initialPos, std::min( pos, cmd.size() ) - initialPos + 1 ) );
            
            
            driver.comm_can_set_pos_spd((uint8_t)std::atoi(inputs[0].c_str()), std::atof(inputs[1].c_str()),
            (uint16_t)std::atoi(inputs[2].c_str()), (uint16_t)std::atoi(inputs[3].c_str()));
            
            //driver.get_data(data);
            // std::cout << "Position: " << data[0] << "\nVelocity: " << data[1] 
            // << "\nTorque: " << data[2] << "\nTemp " << data[3] << "\nError: " << data[4] << "\n"
            // << std::endl;
        }

};




int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleMotor>());
    rclcpp::shutdown();
    

    return 0;
}