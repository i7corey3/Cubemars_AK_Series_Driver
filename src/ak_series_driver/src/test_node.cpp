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


class TestNode : public rclcpp::Node
{

    
    public:


        TestNode()
        : Node("driver")
        { 
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "test_node", 10, std::bind(&TestNode::topic_callback, this, _1));

            publisher_ = this->create_publisher<std_msgs::msg::String>("test_node", 10);
            
        }

        void publisher()
        {
            auto message = std_msgs::msg::String();
            std::string str(data.begin(), data.end());
            message.data = str;
            publisher_->publish(message);
        }
        void setup()
        {
            driver.setup(0, 1000000);
        }

        void read()
        {
            driver.get_data(data);
            printf("Position %f Velocity %f Torque %f", data[0], data[1], data[2]);
        }

    private:
        AKDriver driver;
        std::vector<float> data = {0.0, 0.0, 0.0};

       

        void topic_callback(const std_msgs::msg::String::SharedPtr msg) 
        {
            std::string cmd;
            std::vector<std::string> inputs;
            cmd = msg->data.c_str();
            if (cmd == "start")
            {
                setup();
            }
            else 
            {
                size_t pos = cmd.find(' ');
                size_t initialPos = 0;
                while( pos != std::string::npos)
                {
                    inputs.push_back( cmd.substr( initialPos, pos - initialPos ));
                    initialPos = pos + 1;
                    pos = cmd.find(' ', initialPos);
                    

                }
                inputs.push_back( cmd.substr( initialPos, std::min( pos, cmd.size() ) - initialPos + 1 ) );

                driver.set_motor_param(std::atof(inputs[0].c_str()), std::atof(inputs[1].c_str()), 
                        std::atof(inputs[2].c_str()), std::atof(inputs[3].c_str()), std::atof(inputs[4].c_str()));
                
                driver.get_data(data);
                
            }
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};




int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    TestNode T;
    while (rclcpp::ok())
    {
        rclcpp::spin(std::make_shared<TestNode>());
        T.publisher();
    }
    
    
    rclcpp::shutdown();
    
    // std::vector<std::string> inputs;
    // std::string cmd;
    

   
    //printf("Welcome to the CubMars motor tutorial\n\nStart by typing the motor commands, the inputs are:\n  Desired Position\n  Desired Velocity\n  Desired Torque\n  Kp Value\n  Kd Value\n\n");

    
    // std::cin >> cmd;

    // size_t pos = cmd.find(' ');
    // size_t initialPos = 0;
    // while( pos != std::string::npos)
    // {
    //     inputs.push_back( cmd.substr( initialPos, pos - initialPos ));
    //     initialPos = pos + 1;
    //     pos = cmd.find(' ', initialPos);
        

    // }
    // inputs.push_back( cmd.substr( initialPos, std::min( pos, cmd.size() ) - initialPos + 1 ) );

    // driver.set_motor_param(std::atof(inputs[0].c_str()), std::atof(inputs[1].c_str()), 
    //         std::atof(inputs[2].c_str()), std::atof(inputs[3].c_str()), std::atof(inputs[4].c_str()));
    
    // driver.get_data(data);
    // printf("Position %f, Velocity %f, Torque %f : Inputs %s %s %s %s %s",
    //         data[0], data[1], data[2], inputs[0].c_str(), inputs[1].c_str(), inputs[2].c_str(), inputs[3].c_str(), inputs[4].c_str());
    // inputs.clear();
    //std::cout << data[0] << " " << data[1] << " " << data[2] << " " << std::endl;
                
    
    // CanCommunication can;

    // can.start(0, 1000000);
    // uint8_t msg[8];
    // can.pack_cmd()
    // can.send_cmd(0x01, 0x08, msg);
    
    
    
    // 
    
    return 0;
}