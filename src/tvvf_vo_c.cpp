#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>

#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

int main(int argc, char** argv)
{
    // ROS2の初期化
    rclcpp::init(argc, argv);
    
    try {
        std::cout << "TVVF-VO C++ Node starting..." << std::endl;
        
        // TVVF-VOノードの作成
        auto node = std::make_shared<tvvf_vo_c::TVVFVONode>();
        
        std::cout << "TVVF-VO C++ Node initialized successfully!" << std::endl;
        
        // ノード実行
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
