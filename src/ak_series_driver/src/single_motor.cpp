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
                "control", 10, std::bind(&SingleMotor::topic_callback, this, _1));

            // publisher_ = this->create_publisher<std_msgs::msg::String>("test_node", 10);
            // std::thread t(&SingleMotor::read, this);
           
            setup();
            
            
        }
     
        void setup()
        {
            driver.setup(0, 1000000);
            //driver.reset_position(addr);
            // 
            // driver.setup_motor("motor1", 0, 0.0, 17.0);
            printf("Setup Complete\r\n");
            driver.comm_can_set_pos_spd(0x68, 10.0, 10000, 10000);
            
            //driver.comm_can_set_rpm(0x68, 20000);
            
            //driver.start_motor(addr);
            
        }

 

    private:
        AKDriver driver;
        int addr = 0;
        std::vector<float> data = {0.0, 0.0, 0.0, 0.0, 0.0};
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       
        void read()
        {
            while (1)
            {
               
                
            }
        } 

        void topic_callback(const std_msgs::msg::String::SharedPtr msg) 
        {
            
            std::string cmd;
            std::vector<std::string> inputs;
            cmd = msg->data.c_str();
            if (cmd == "start")
            {
                driver.start_motor(addr);
            }
            else if (cmd == "stop")
            {
                driver.stop_motor(addr);
            }
            else if (cmd == "reset")
            {
                driver.reset_position(addr);
            }
            
            else if (cmd == "")
            {
               
               
               driver.start_motor(addr);
                //  driver.set_motor_param(std::atof(stored_inputs[0].c_str()), std::atof(stored_inputs[1].c_str()), 
                //     std::atof(stored_inputs[2].c_str()), std::atof(stored_inputs[3].c_str()), std::atof(stored_inputs[4].c_str()));
                
            }
            else 
            {
                
                try 
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
                    
                    
                   // driver.move_to_position(addr, std::atof(inputs[0].c_str()));
                    driver.set_motor_param(addr, std::atof(inputs[0].c_str()), std::atof(inputs[1].c_str()), 
                        std::atof(inputs[2].c_str()), std::atof(inputs[3].c_str()), std::atof(inputs[4].c_str()));
                    driver.start_motor(addr);
                }

                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                    driver.start_motor(addr);
                }
                
            }
            
            driver.get_data(data);
            std::cout << "Position: " << data[0] << "\nVelocity: " << data[1] 
            << "\nTorque: " << data[2] << "\nTemp " << data[3] << "\nError: " << data[4] << "\n" 
            << std::endl;
            std::cout << driver.motor1_.angle << std::endl;
            //printf("Position %f Velocity %f Torque %f", data[0], data[1], data[2]);
            
        }
        

};




int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleMotor>());
    rclcpp::shutdown();
    

    
    return 0;
}