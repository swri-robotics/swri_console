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

#ifndef SWRI_CONSOLE_CONSOLE_MASTER_H_
#define SWRI_CONSOLE_CONSOLE_MASTER_H_

#include <string>
#include <QObject>
#include <QList>
#include <QFont>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <swri_console/log_database.h>
#include <swri_console/bag_reader.h>
#include <swri_console/rosout_log_loader.h>
#include <swri_console/ros_thread.h>

namespace swri_console
{
typedef std::vector<rcl_interfaces::msg::Log::SharedPtr> MessageList;

class ConsoleWindow;
class ConsoleMaster : public QObject
{
  Q_OBJECT;

 public:  
  ConsoleMaster(int argc, char** argv);
  ~ConsoleMaster() override;

 public Q_SLOTS:
  void createNewWindow();
  void fontSelectionChanged(const QFont &font);
  void selectFont();

 Q_SIGNALS:
  void fontChanged(const QFont &font);

 private:
  BagReader bag_reader_;
  RosoutLogLoader log_reader_;

  // All ROS operations are done on a separate thread to ensure they do not
  // cause the GUI thread to block.
  RosThread ros_thread_;
  
  bool connected_;

  QList<ConsoleWindow*> windows_;

  LogDatabase db_;

  QFont window_font_;
};  // class ConsoleMaster
}  // namespace swri_console
#endif  // SWRI_CONSOLE_CONSOLE_MASTER_H_
