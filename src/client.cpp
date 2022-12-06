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

#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>


#include "beginner_tutorials/srv/count.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Declare service client node
  std::shared_ptr<rclcpp::Node> node = \
  rclcpp::Node::make_shared("client_for_updating_string");
  // Bind client to callback
  rclcpp::Client<beginner_tutorials::srv::Count>::SharedPtr client =
    node->create_client<beginner_tutorials::srv::Count>\
    ("minimal_publisher/Count");
  // Get the request of count
  auto request = std::make_shared<beginner_tutorials::srv::Count::Request>();

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), \
      "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), \
    "service not available, waiting again...");
  }

  // Send the request
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Count is: [%s] ",\
    std::to_string(result.get()->count).c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  rclcpp::shutdown();
  return 0;
}