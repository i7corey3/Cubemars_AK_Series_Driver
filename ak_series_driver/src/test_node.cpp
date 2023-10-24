#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ak_series_driver/mit_params.hpp"


#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

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

    std::cout << ERROR_CODES(6) << std::endl;
    
    // rclcpp::spin(std::make_shared<TestNode>());
    
    
    // rclcpp::shutdown();
    
    return 0;
}