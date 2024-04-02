#include <functional>
#include <string>
#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>


#include <fstream>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cstdio>
#include "show_path_interfaces/srv/manual_target.hpp"

#include <unistd.h> 


using namespace std::chrono_literals;
using ManualTarget = show_path_interfaces::srv::ManualTarget;
using namespace std;

const int lens = 210;
float position[lens][3] = {0};
int counter = 0;

float x_pos, y_pos,z_pos=0;
float yaw = 0;
float x_next, y_next,yaw_next=0;
float error = 0;


geometry_msgs::msg::Quaternion q_ori;

auto createQuaternionMsgFromYaw(float yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}



nav_msgs::msg::Path plot_path;
class ControlNode : public rclcpp::Node
{
  public:
    ControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
    : Node("control_node", options), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("trajectory",10);
      timer_ =  this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&ControlNode::timer_callback, this)); // Modified the function name to use 'Publisher' instead of 'ControlNode'
      plot_path.header.stamp = this->get_clock()->now();
      plot_path.header.frame_id = "map";

      client = this->create_client<show_path_interfaces::srv::ManualTarget>("manual_target1");

      client_timer_ = this->create_wall_timer(
      5s,
      [this]() {
        std::vector<int64_t> pruned_requests;
        // Prune all requests older than 5s.
        size_t n_pruned = this->client->prune_requests_older_than(
          std::chrono::system_clock::now() - 5s, &pruned_requests);
        if (n_pruned) {
          RCLCPP_INFO(
            this->get_logger(),
            "The server hasn't replied for more than 5s, %zu requests were discarded, "
            "the discarded requests numbers are:",
            n_pruned);
          for (const auto & req_num : pruned_requests) {
            RCLCPP_INFO(this->get_logger(), "\t%" PRId64, req_num);
          }
        }
      });


    }
  bool
  wait_for_service_server()
  {
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return false;
      }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    return true;
  }

  void
  queue_async_request(float a, float b, float yaw)
  {
    auto request = std::make_shared<ManualTarget::Request>();
    request->x_now = a;
    request->y_now = b;
    request->yaw_now = yaw;

    // We give the async_send_request() method a callback that will get executed once the response
    // is received.
    // This way we can return immediately from this method and allow other work to be done by the
    // executor in `spin` while waiting for the response.
    using ServiceResponseFuture =
      rclcpp::Client<ManualTarget>::SharedFutureWithRequest;
    auto response_received_callback =
      [logger = this->get_logger()](ServiceResponseFuture future) {
        auto request_response_pair = future.get();
        RCLCPP_INFO(
          logger,
          // "Result of %" PRId64 " + %" PRId64 " is: %" PRId64,
          // request_response_pair.first->a,
          // request_response_pair.first->b,
          // request_response_pair.second->sum
          "response is: %f %f %f",
           x_next = request_response_pair.second->x_next,
           y_next = request_response_pair.second->y_next,
           yaw_next = request_response_pair.second->yaw_next);
      };
    auto result = client->async_send_request(
      request, std::move(response_received_callback));
    RCLCPP_INFO(
      this->get_logger(),
      "Sending a request to the server (request_id =%" PRId64
      "), we're going to let you know the result when ready!",
      result.request_id);
  }

  private:
    void timer_callback()
    {
        
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
        RCLCPP_INFO(this->get_logger(),"x_next=%.2f",x_next);
        RCLCPP_INFO(this->get_logger(),"y_next=%.2f",y_next);
        RCLCPP_INFO(this->get_logger(),"distance=%.2f",sqrt(plot_pose_stamped.pose.position.x*plot_pose_stamped.pose.position.x+plot_pose_stamped.pose.position.y*plot_pose_stamped.pose.position.y));
        error = sqrt((x_next - x_pos)*(x_next - x_pos)+(y_next - y_pos)*(y_next - y_pos));
        RCLCPP_INFO(this->get_logger(),"error=%.2f",error);
        RCLCPP_INFO(this->get_logger()," ");
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    size_t count_;
    rclcpp::Client<show_path_interfaces::srv::ManualTarget>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr client_timer_;
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
    // rclcpp::spin(std::make_shared<ControlNode>());

    auto node = std::make_shared<ControlNode>();
    const auto & logger = node->get_logger();
    RCLCPP_INFO(logger, "waiting for service to appear...");
    if (!node->wait_for_service_server()) {
      return 1;
    }
    RCLCPP_INFO(logger, "Update target server is available!!!");

    std::promise<void> stop_async_spinner;
    std::thread async_spinner_thread(
    [stop_token = stop_async_spinner.get_future(), node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      executor.spin_until_future_complete(stop_token);
    });
  
    while(1){

      if (counter < lens){
        x_pos = position[counter][0];
        y_pos = position[counter][1];
        yaw = position[counter][2];
        counter++;
      }
      else{
        counter = 0;
      }
      node->queue_async_request(x_pos, y_pos, yaw);
      usleep(1000*1000);

    }

    rclcpp::shutdown();
    return 0;

}
