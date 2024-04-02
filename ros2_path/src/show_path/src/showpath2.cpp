#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

auto createQuaternionMsgFromYaw(float yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}
float x = 0.0;
float y = 0.0;
float th = 0.0;
float vx = 0.1;
float vy = -0.1;
float vth = 0.1;
float x_plot, y_plot;
float f = 0.0;
uint32_t i = 0;
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


      float x_plot = 5 * sin(f + i / 100.0f * 2 * M_PI);
			float y_plot = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::msg::PoseStamped plot_pose_stamped;
      plot_pose_stamped.pose.position.x = x_plot;
      plot_pose_stamped.pose.position.y = y_plot;
      plot_pose_stamped.pose.position.z = 0;
      plot_pose_stamped.pose.orientation.x = 1;
      plot_pose_stamped.pose.orientation.y = 0;
      plot_pose_stamped.pose.orientation.z = 0;
      plot_pose_stamped.pose.orientation.w = 1;

      plot_pose_stamped.header.stamp=this->get_clock()->now();
      plot_pose_stamped.header.frame_id="map";
      plot_path.poses.push_back(plot_pose_stamped);
      publisher_->publish(plot_path);



      f += 0.04;
      if (i == 100 || i>100){
        i = 0;
      } 
      else {
        i++;
      }
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    size_t count_;
};

int main (int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;

}
