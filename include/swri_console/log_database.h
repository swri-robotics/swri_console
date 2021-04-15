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

#ifndef SWRI_CONSOLE_LOG_DATABASE_H_
#define SWRI_CONSOLE_LOG_DATABASE_H_

#include <QObject>
#include <QAbstractListModel>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <deque>
#include <rclcpp/time.hpp>

namespace swri_console
{
struct LogEntry
{
  rclcpp::Time stamp;
  uint8_t level_mask;
  std::string node;
  std::string file;
  std::string function;
  uint32_t line;
  QStringList text;
  uint32_t seq;

  void setLogLvl(uint8_t ros_log_lvl)
  {
    level = ros_log_lvl;
    switch (ros_log_lvl) {
      case rcl_interfaces::msg::Log::DEBUG : level_mask = 0x01; break;
      case rcl_interfaces::msg::Log::INFO : level_mask = 0x02; break;
      case rcl_interfaces::msg::Log::WARN : level_mask = 0x04; break;
      case rcl_interfaces::msg::Log::ERROR : level_mask = 0x08; break;
      case rcl_interfaces::msg::Log::FATAL : level_mask = 0x10; break;
      default: level_mask = 0x01; break;
    }
  }

  uint8_t getLogLvl(void) const { return level; }

private:
  uint8_t level;
};

class LogDatabase : public QObject
{
  Q_OBJECT
  
public:
  LogDatabase();
  ~LogDatabase() override = default;
  
  void clear();
  const std::deque<LogEntry>& log() { return log_; }
  const rclcpp::Time& minTime() const { return min_time_; }

  const std::map<std::string, size_t>& messageCounts() const { return msg_counts_; }

 Q_SIGNALS:
  void databaseCleared();
  void messagesAdded();
  void minTimeUpdated();

public Q_SLOTS:
  void queueMessage(const rcl_interfaces::msg::Log::ConstSharedPtr msg);
  void processQueue();

private:  
  std::map<std::string, size_t> msg_counts_;
  std::deque<LogEntry> log_;
  std::deque<LogEntry> new_msgs_;

  rclcpp::Time min_time_;
};  // class LogDatabase
}  // namespace swri_console 
#endif  // SWRI_CONSOLE_LOG_DATABASE_H_
