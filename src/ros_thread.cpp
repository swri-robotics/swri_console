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
#include "include/swri_console/ros_thread.h"

using namespace swri_console;

RosThread::RosThread(int argc, char** argv) :
  is_connected_(false),
  is_running_(true)
{
  ros::init(argc, argv, "swri_console",
            ros::init_options::AnonymousName |
            ros::init_options::NoRosout);
}

void RosThread::run()
{
  while (is_running_)
  {
    bool master_status = ros::master::check();

    if (!is_connected_ && master_status) {
      startRos();
    } else if (is_connected_ && !master_status) {
      stopRos();
    } else if (is_connected_ && master_status) {
      ros::spinOnce();
      Q_EMIT spun();
    }
    msleep(50);
  }
}


void RosThread::shutdown()
{
  is_running_ = false;
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void RosThread::startRos()
{
  ros::start();
  is_connected_ = true;

  ros::NodeHandle nh;
  rosout_sub_ = nh.subscribe("/rosout_agg", 10000,
                             &RosThread::handleRosout,
                             this);
  Q_EMIT connected(true);
}

void RosThread::stopRos()
{
  ros::shutdown();
  is_connected_ = false;
  Q_EMIT connected(false);
}

void RosThread::handleRosout(const rosgraph_msgs::LogConstPtr &msg)
{
  Q_EMIT logReceived(msg);
}
