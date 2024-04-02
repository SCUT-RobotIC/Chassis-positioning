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


#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

float x_plot, y_plot, z_plot;
const int lens = 210;
float position[lens][3] = {0};
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
        if (count_ < lens)
        {

            geometry_msgs::msg::PoseStamped plot_pose_stamped;
            plot_pose_stamped.pose.position.x = position[count_][0];
            plot_pose_stamped.pose.position.y = position[count_][1];
            plot_pose_stamped.pose.position.z = z_plot;
            plot_pose_stamped.pose.orientation.x = 0;
            plot_pose_stamped.pose.orientation.y = 0;
            plot_pose_stamped.pose.orientation.z = 0;
            plot_pose_stamped.pose.orientation.w = 1;

            plot_pose_stamped.header.stamp=this->get_clock()->now();
            plot_pose_stamped.header.frame_id="map";
            plot_path.poses.push_back(plot_pose_stamped);
            publisher_->publish(plot_path);

            RCLCPP_INFO(this->get_logger(),"x_plot=%.2f",plot_pose_stamped.pose.position.x);
            RCLCPP_INFO(this->get_logger(),"y_plot=%.2f",plot_pose_stamped.pose.position.y);
            RCLCPP_INFO(this->get_logger(),"distance=%.2f",sqrt(plot_pose_stamped.pose.position.x*plot_pose_stamped.pose.position.x+plot_pose_stamped.pose.position.y*plot_pose_stamped.pose.position.y));
            RCLCPP_INFO(this->get_logger()," ");
            count_++;
        }

        

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    size_t count_;
};

int main (int argc, char **argv)
{
    ifstream selected_robot("/home/peter/ros2_path/test_data6.txt");
    // ifstream selected_robot("/home/peter/ros2_path/local0304.txt");
        if (!selected_robot.is_open()){
            cout << "can not open selected_robot file" << endl;
        }
        for (int i = 0; i < lens; i++){
            for (int j = 0; j < 3; j++){
                selected_robot >> position[i][j];
            }
        }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;

}
