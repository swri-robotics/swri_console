// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <QCoreApplication>
#include "swri_console/ros_thread.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace swri_console;

using namespace std::literals::chrono_literals;

RosThread::RosThread(int argc, char** argv) :
  is_connected_(false),
  is_running_(true)
{
  rclcpp::init(argc, argv);
}

void RosThread::run()
{
  while (is_running_)
  {
    bool is_initialized = rclcpp::is_initialized();

    if (!is_connected_ && is_initialized) {
      startRos();
    } else if (is_connected_ && !is_initialized) {
      stopRos();
    } else if (is_connected_ && is_initialized) {
      rclcpp::spin_some(nh_);
      Q_EMIT spun();
    }
    msleep(50);
  }
}


void RosThread::shutdown()
{
  is_running_ = false;
  if (rclcpp::is_initialized())
  {
    rclcpp::shutdown();
  }
}

void RosThread::startRos()
{
  is_connected_ = true;

  // ROS 2 Dashing doesn't support making an anonymous name as an init option,
  // so we manually make it anonymous.  This is the same way ros::init does
  // it in ROS 1.
  std::stringstream name;
  name << "swri_console";
  char buf[200];
  std::snprintf(buf, sizeof(buf), "_%llu", (unsigned long long)rclcpp::Clock().now().nanoseconds());
  name << buf;

  nh_ = rclcpp::Node::make_shared(name.str());

  rosout_sub_ = nh_->create_subscription<rcl_interfaces::msg::Log>("/rosout", 100,
      [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
    Q_EMIT logReceived(std::move(msg));
  });

  Q_EMIT connected(true);
}

void RosThread::stopRos()
{
  rclcpp::shutdown();
  is_connected_ = false;
  Q_EMIT connected(false);
}
