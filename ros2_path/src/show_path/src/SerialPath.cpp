#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "Serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <iostream>
#include <cstdio>

float x_pos, y_pos,z_pos=0;
float yaw = 0;
// float x_ori= 0;
// float y_ori= 0;
// float z_ori= 0;
// float w_ori= 1;
// float x_vel= 0;
// float y_vel= 0;
// float z_vel= 0;
// float x_ang= 0;
// float y_ang= 0;
// float z_ang= 0;
// float x_acc= 0;
// float y_acc= 0;
// float z_acc= 0;

Serial s1;
std::string str = "/dev/ttyS0";   //串口号
geometry_msgs::msg::Quaternion q_ori;

auto createQuaternionMsgFromYaw(float yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}



nav_msgs::msg::Path plot_path;
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("trajectory",10);
      timer_ =  this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&MinimalPublisher::timer_callback, this)); // Modified the function name to use 'Publisher' instead of 'MinimalPublisher'
      plot_path.header.stamp = this->get_clock()->now();
      plot_path.header.frame_id = "map";
    }

  private:
    void timer_callback()
    {
        unsigned char bf[100] = {0};
        int len = s1.Recv(bf, sizeof(bf));
        // 将接收到的数据转换为字符串
        std::string data(reinterpret_cast<char*>(bf));

        // 创建一个输入字符串流
        std::istringstream iss(data);

        // 从输入字符串流中解析浮点数
        // iss >> x_pos >> y_pos >> x_ori >> y_ori >> z_ori >> w_ori >> x_ang >> y_ang >> z_ang >> x_acc >> y_acc >> z_acc;
        iss >> x_pos >> y_pos >> yaw;



        geometry_msgs::msg::PoseStamped plot_pose_stamped;
        plot_pose_stamped.pose.position.x = x_pos;
        plot_pose_stamped.pose.position.y = y_pos;
        plot_pose_stamped.pose.position.z = z_pos;
        q_ori = createQuaternionMsgFromYaw(yaw);
        plot_pose_stamped.pose.orientation.x = q_ori.x;
        plot_pose_stamped.pose.orientation.y = q_ori.y;
        plot_pose_stamped.pose.orientation.z = q_ori.z;
        plot_pose_stamped.pose.orientation.w = q_ori.w;

        plot_pose_stamped.header.stamp=this->get_clock()->now();
        plot_pose_stamped.header.frame_id="map";
        plot_path.poses.push_back(plot_pose_stamped);
        publisher_->publish(plot_path);
        RCLCPP_INFO(this->get_logger(),"x_pos=%.2f",plot_pose_stamped.pose.position.x);
        RCLCPP_INFO(this->get_logger(),"y_pos=%.2f",plot_pose_stamped.pose.position.y);
        RCLCPP_INFO(this->get_logger(),"distance=%.2f",sqrt(plot_pose_stamped.pose.position.x*plot_pose_stamped.pose.position.x+plot_pose_stamped.pose.position.y*plot_pose_stamped.pose.position.y));
        RCLCPP_INFO(this->get_logger()," ");

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    size_t count_;
};

int main (int argc, char **argv)
{
    s1.OpenSerial(str, E_BaudRate::_115200, E_DataSize::_8, E_Parity::None, E_StopBit::_1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;

}
