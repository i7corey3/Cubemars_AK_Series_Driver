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


using std::placeholders::_1;
using namespace std::chrono_literals;


unsigned char find_can(const int port)
{
	char buf[128]={0};
	sprintf(buf,"/sys/class/net/can%d/can_bittiming/bitrate",port);
	return ((access(buf,0)==0));
}


class DualMotor : public rclcpp::Node
{

    
    public:

        DualMotor()
        : Node("dual_motor")
        { 
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "control", 10, std::bind(&DualMotor::topic_callback, this, _1));

            setup();
            
            
        }
     
        void setup()
        {
            driver.setup(0, 1000000);
            driver.reset_position(0);
            driver.start_motor(0);
            driver.reset_position(1);
            driver.start_motor(1);
        }

 

    private:
        AKDriver driver;
        int addr = 1;
        std::vector<float> data = {0.0, 0.0, 0.0, 0.0, 0.0};
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       

        void topic_callback(const std_msgs::msg::String::SharedPtr msg) 
        {
            
            std::string cmd;
            std::vector<std::string> inputs;
            cmd = msg->data.c_str();
            if (cmd == "m1")
            {
                addr = 0;
            }
            else if (cmd == "m2")
            {
                addr = 1;
            }
            else if (cmd == "start")
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
                    // for (int i=0; i < 5; i++)
                    // {
                    //     stored_inputs[i] = inputs[i];
                    // }
                    
                    
                    driver.set_motor_param(addr, std::atof(inputs[0].c_str()), std::atof(inputs[1].c_str()), 
                        std::atof(inputs[2].c_str()), std::atof(inputs[3].c_str()), std::atof(inputs[4].c_str()));
                    
                }

                catch(const std::exception& e)
                {
                    //std::cerr << e.what() << '\n';
                    driver.start_motor(addr);
                }
                
            }
            driver.get_data(data);
            std::cout << "Position: " << data[0] << "\nVelocity: " << data[1] 
            << "\nTorque: " << data[2] << "\nTemp " << data[3] << "\nError: " << data[4] << "\n"
            << std::endl;
            //printf("Position %f Velocity %f Torque %f", data[0], data[1], data[2]);
            
        }
        

};




int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualMotor>());
    rclcpp::shutdown();
    

    
    return 0;
}