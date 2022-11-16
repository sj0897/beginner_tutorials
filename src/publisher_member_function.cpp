// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node{
 public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0),ctr_(1000) {
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
      
  }

 private:
  void timer_callback() {
    ctr_ = this->get_parameter("count").get_parameter_value().get<int>();
    if(count_ < ctr_){
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! This is Sparsh's Publisher "
                    + std::to_string(count_++);
    switch ((count_-1)%5) {
    case 0:
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:"<< message.data.c_str());
      break;
    
    case 1:
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing:"<< message.data.c_str());
      break;
    
    case 2:
      RCLCPP_WARN_STREAM(this->get_logger(), "Publishing:"<< message.data.c_str());
      break;
    
    case 3:
      RCLCPP_ERROR_STREAM(this->get_logger(), "Publishing:"<< message.data.c_str());
      break;
    
    case 4:
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publishing:"<< message.data.c_str());
      break;
    }
    publisher_->publish(message);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int count_;
  int ctr_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
