#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>


#include "beginner_tutorials/srv/count.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using namespace std::placeholders;
using Count = beginner_tutorials::srv::Count;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node{
 public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0), ctr_(10) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

    if (rcutils_logging_set_logger_level(this->get_logger().get_name(),
                      RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG)
          == RCUTILS_RET_OK) {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Set logger level DEBUG success.");
      } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "Set logger level DEBUG fails.");
      }
      this->declare_parameter("count", ctr_);

    std::string get_count_service_name =
        "/" + std::string(this->get_name()) + "/" + "Count";
    get_count_service_ = this->create_service<Count>(
        get_count_service_name,
        std::bind(&MinimalPublisher::get_count_callback, this, _1, _2));
          
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Timer constantly publishing tf info
      tf_timer_ = this->create_wall_timer(
          200ms, std::bind(&MinimalPublisher::broadcast_timer_callback, this));
  }

 private:
  void timer_callback() {
    ctr_ = this->get_parameter("count").get_parameter_value().get<int>();
    if (count_ < ctr_) {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! This is Sparsh's Publisher "
                    + std::to_string(count_++);
    switch ((count_-1)%5) {
    case 0:
      RCLCPP_INFO_STREAM(this->get_logger(),
                          "Publishing:"<< message.data.c_str());
      break;

    case 1:
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                           "Publishing:"<< message.data.c_str());
      break;

    case 2:
      RCLCPP_WARN_STREAM(this->get_logger(),
                           "Publishing:"<< message.data.c_str());
      break;

    case 3:
      RCLCPP_ERROR_STREAM(this->get_logger(),
                           "Publishing:"<< message.data.c_str());
      break;

    case 4:
      RCLCPP_FATAL_STREAM(this->get_logger(),
                           "Publishing:"<< message.data.c_str());
      break;
    }
    publisher_->publish(message);
    }
  }

  void get_count_callback(const std::shared_ptr<Count::Request> request,
                          std::shared_ptr<Count::Response> response) {
    (void)request;
    response->count = ctr_;
  }

  void broadcast_timer_callback() {
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0, 0, 1.57);

      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world";
      t.child_frame_id = "talk";
      t.transform.translation.x = 1.0;
      t.transform.translation.y = 2.0;
      t.transform.translation.z = 3.0;
      t.transform.rotation.x = tf2_quat.x();
      t.transform.rotation.y = tf2_quat.y();
      t.transform.rotation.z = tf2_quat.z();
      t.transform.rotation.w = tf2_quat.w();

      tf_broadcaster_->sendTransform(t);
      RCLCPP_INFO_STREAM(this->get_logger(),
                          "Transform Published");
    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<Count>::SharedPtr get_count_service_;
  int count_;
  int ctr_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
  rclcpp::TimerBase::SharedPtr tf_timer_;
};