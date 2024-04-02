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


nav_msgs::msg::Path plot_path;
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
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("trajectory",5);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), [this](){timer_callback();});
      plot_path.header.stamp = this->get_clock()->now();
      plot_path.header.frame_id = "map";
    }

  private:
    void timer_callback()
    {
      float dt = 0.1;
      float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
      float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
      float delta_th = vth * dt;

      
      x_plot += delta_x;
      y_plot += delta_y;
      th += delta_th;
      
      geometry_msgs::msg::PoseStamped plot_pose_stamped;
      plot_pose_stamped.pose.position.x = x_plot;
      plot_pose_stamped.pose.position.y = y_plot;
      plot_pose_stamped.pose.position.z = 0;
      

      geometry_msgs::msg::Quaternion goal_quat = createQuaternionMsgFromYaw(th);
      plot_pose_stamped.pose.orientation.x = goal_quat.x;
      plot_pose_stamped.pose.orientation.y = goal_quat.y;
      plot_pose_stamped.pose.orientation.z = goal_quat.z;
      plot_pose_stamped.pose.orientation.w = goal_quat.w;

      plot_pose_stamped.header.stamp = this->get_clock()->now();
      plot_pose_stamped.header.frame_id = "odom";
      plot_path.poses.push_back(plot_pose_stamped);
      publisher_->publish(plot_path);

      RCLCPP_INFO(this->get_logger(),"x_plot=%.2f",plot_pose_stamped.pose.position.x);
      RCLCPP_INFO(this->get_logger(),"y_plot=%.2f",plot_pose_stamped.pose.position.y);
      RCLCPP_INFO(this->get_logger(),"z_plot=%.2f",plot_pose_stamped.pose.position.z);
      RCLCPP_INFO(this->get_logger(),"orientation.x=%.2f",plot_pose_stamped.pose.orientation.x);
      RCLCPP_INFO(this->get_logger(),"orientation.y=%.2f",plot_pose_stamped.pose.orientation.y);
      RCLCPP_INFO(this->get_logger(),"orientation.z=%.2f",plot_pose_stamped.pose.orientation.z);
      RCLCPP_INFO(this->get_logger(),"orientation.w=%.2f",plot_pose_stamped.pose.orientation.w);
      RCLCPP_INFO(this->get_logger()," ");
      
      
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
