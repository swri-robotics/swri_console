// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#ifndef SWRI_CONSOLE_ROS_THREAD_H
#define SWRI_CONSOLE_ROS_THREAD_H

#include <QThread>

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <QMetaType>

namespace swri_console
{
  class RosThread : public QThread
  {
    Q_OBJECT
  public:
    RosThread(int argc, char** argv);
    /*
     * Shuts down ROS and causes the thread to exit.
     */
    void shutdown();

  Q_SIGNALS:
    /**
     * Emitted every time we are successfully connected to or disconnected from ROS.
     */
    void connected(bool);
    /**
     * Emitted with every batch of log messages drained from the ROS subscription queue
     * in a single spin.  This can be emitted multiple times per spin of the ROS core;
     * wait until spun() is emitted to do any processing on them.
     */
    void logReceived(std::vector<rcl_interfaces::msg::Log::ConstSharedPtr> msgs);
    /**
     * Emitted after every time ros::spinOnce() completes.
     */
    void spun();

  protected:
    void run() override;

  private:
    void startRos();
    void stopRos();

    rclcpp::QoS getQos();
    void emptyLogQueue(rcl_interfaces::msg::Log::ConstSharedPtr msg);

    bool is_connected_;
    volatile bool is_running_;

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  };
}

#endif //SWRI_CONSOLE_ROS_THREAD_H
