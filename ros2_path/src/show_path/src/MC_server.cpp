#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "Serial.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
#include "show_path_interfaces/srv/manual_target.hpp"
float x, y,yaw=0;
using ManualTarget = show_path_interfaces::srv::ManualTarget;
class McSeverNode : public rclcpp::Node
{
public:
    McSeverNode() : Node("MC_server")
     {
        // 创建第一个服务
        service1_ = this->create_service<ManualTarget>("manual_target1",
            std::bind(&McSeverNode::handle_service1, this, std::placeholders::_1, std::placeholders::_2));

        // 创建第二个服务
        service2_ = this->create_service<ManualTarget>("manual_target2",
            std::bind(&McSeverNode::handle_service2, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_service1(const std::shared_ptr<ManualTarget::Request> request,
                         std::shared_ptr<ManualTarget::Response> response)
    {
        response->x_next= x;
        response->y_next= y;
        response->yaw_next= yaw;
        // RCLCPP_INFO(this->get_logger(), "Auto update: %f %f %f", response->x_next, response->y_next, response->yaw_next);
    }

    void handle_service2(const std::shared_ptr<ManualTarget::Request> request,
                         std::shared_ptr<ManualTarget::Response> response)
    {
        x = request->x_now;
        y = request->y_now;
        yaw = request->yaw_now;

        response->x_next= x;
        response->y_next= y;
        response->yaw_next= yaw;
        RCLCPP_INFO(this->get_logger(), "Manual update: %f %f %f", response->x_next, response->y_next, response->yaw_next);
    }

    rclcpp::Service<ManualTarget>::SharedPtr service1_;
    rclcpp::Service<ManualTarget>::SharedPtr service2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<McSeverNode>());
    rclcpp::shutdown();
    return 0;
}