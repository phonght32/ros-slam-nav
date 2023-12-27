// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Author: Darby Lim

#include "robot_node/devices/sound.hpp"

#include <memory>
#include <string>

using robotis::robot::devices::Sound;

Sound::Sound(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
: Devices(nh, dxl_sdk_wrapper)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create sound server");
  srv_ = nh_->create_service<robot_msgs::srv::Sound>(
    server_name,
    [this](
      const std::shared_ptr<robot_msgs::srv::Sound::Request> request,
      std::shared_ptr<robot_msgs::srv::Sound::Response> response) -> void
    {
      this->command(static_cast<void *>(request.get()), static_cast<void *>(response.get()));
    }
  );
}

void Sound::command(const void * request, void * response)
{
  robot_msgs::srv::Sound::Request req = *(robot_msgs::srv::Sound::Request *)request;
  robot_msgs::srv::Sound::Response * res = (robot_msgs::srv::Sound::Response *)response;

  res->success = dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.sound.addr,
    extern_control_table.sound.length,
    reinterpret_cast<uint8_t *>(&req.value),
    &res->message);
}

void Sound::request(
  rclcpp::Client<robot_msgs::srv::Sound>::SharedPtr client,
  robot_msgs::srv::Sound::Request req)
{
  auto request = std::make_shared<robot_msgs::srv::Sound::Request>(req);
  auto result = client->async_send_request(request);
}
