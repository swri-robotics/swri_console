// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <QFileDialog>
#include <QDir>
#include <QDirIterator>

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <swri_console/rosout_log_loader.h>
#include <ctime>
#include <string>

namespace swri_console
{
  const int MIN_MSG_SIZE=10;

  void RosoutLogLoader::loadRosLogDirectory(const QString& logdirectory_name)
  {
    QDirIterator it(logdirectory_name, QStringList() << "*.log", QDir::Files);
    while (it.hasNext())
    {
        QString filename = it.next();
        printf("Loading log file %s ...\n", filename.toStdString().c_str());
        loadRosLog(filename);
    }
  }

  void RosoutLogLoader::loadRosLog(const QString& logfile_name)
  {
    std::string std_string_logfile = logfile_name.toStdString();
    std::ifstream logfile(std_string_logfile.c_str());
    for( std::string line; getline( logfile, line ); )
    {
      rcl_interfaces::msg::Log log;
      unsigned found = std_string_logfile.find_last_of("/\\");
      log.name = std_string_logfile.substr(found+1);
      int result = parseLine(line, &log);
      if (result == 0)
      {
        rcl_interfaces::msg::Log::SharedPtr log_ptr = std::make_shared<rcl_interfaces::msg::Log>(log);
        emit logReceived(log_ptr);
      }
    }
    emit finishedReading();
  }

  int RosoutLogLoader::parseLine(const std::string& line, rcl_interfaces::msg::Log* log)
  {
    // Example: 1507066364.728102032 INFO [/home/pwesthart/code/src/mapviz/tile_map/src/tile_map_plugin.cpp:260(TileMapPlugin::PrintInfo) [topics: /rosout] OK
    char log_msg_fmt0[] = "%d.%d %s [%[^:]:%u(%[^)]) [topics: %[^]]] %[^\n]s";
    int secs = 0;
    int nsecs = 0;
    char level[16];
    char file[1024];
    unsigned int line_num = 1;
    char function[128];
    char topics[1024];
    char msg[1024*32];
    rclcpp::Time stamp;

    /// Scan variables in from parsed line
    int num_parsed = sscanf(line.c_str(),log_msg_fmt0,&secs, &nsecs, level, file, &line_num, function, topics, msg);
    if (num_parsed == 8 )
    {
      // Populate new log message
      log->file = file;
      log->function = function;
      // log->header.seq = seq;
      // stamp.sec = secs;
      // stamp.nsec = nsecs;
      stamp = rclcpp::Time(secs, nsecs);
      log->stamp = stamp;
      log->level = level_string_to_level_type(std::string(level));
      log->line = line_num;
      log->msg = msg;
    }
    else // try another format
    {
      char log_msg_fmt1[] = "%d.%d %s [%[^:]:%u(%[^)])) [topics: %[^]]] %[^\n]s";
      num_parsed = sscanf(line.c_str(), log_msg_fmt1, &secs, &nsecs, level, file, &line_num, function, topics, msg);
      if (num_parsed == 8 )
      {
        // Populate new log message
        log->file = file;
        log->function = function;
        // log->header.seq = seq;
        // stamp.sec = secs;
        // stamp.nsec = nsecs;
        stamp = rclcpp::Time(secs, nsecs);
        log->stamp = stamp;
        log->level = level_string_to_level_type(std::string(level));
        log->line = line_num;
        log->msg = msg;
      }
      else // try another format
      {
        // Example: [rospy.registration][INFO] 2017-11-30 08:11:39,231: registering subscriber topic [/tf] type [tf2_msgs/TFMessage] with master
        char log_msg_fmt2[] = "[%[^]]][%[^]]] %d-%d-%d %d:%d:%d,%d: %[^\n]s";
        int year;
        int month;
        int day;
        int hour;
        int minute;
        int msecs;
        char name[1024];
        time_t rawtime;
        struct tm * timeinfo;

        num_parsed = sscanf(line.c_str(), log_msg_fmt2, name, level, &year, &month, &day, &hour, &minute, &secs, &msecs, msg);
        if (num_parsed == 10)
        {
          // Populate new log message
          file[0] = 0;
          function[0] = 0;
          line_num = 0;
          time ( &rawtime );
          timeinfo = localtime ( &rawtime );
          timeinfo->tm_year = year - 1900;
          timeinfo->tm_mon = month - 1;
          timeinfo->tm_mday = day;
          timeinfo->tm_hour = hour;
          timeinfo->tm_min = minute;
          timeinfo->tm_sec = secs;
          rawtime = mktime ( timeinfo );
          secs = rawtime;
          nsecs = msecs * 1000000;
          log->file = file;
          log->function = function;
          // log->header.seq = seq;
          // stamp.sec = secs;
          // stamp.nsec = nsecs;
          stamp = rclcpp::Time(secs, nsecs);
          log->stamp = stamp;
          log->level = level_string_to_level_type(std::string(level));
          log->line = line_num;
          log->msg = msg;
        }
        else
        {
          // Example: [ INFO] [1512051098.518631473]: Read parameter lower_cost_threshold = 0.000000
          char log_msg_fmt3[] = "\x1b[%dm[ %[^]]] [%d.%d]: %[^\n\x1b]s";
//          char log_msg_fmt3[] = "\x1b[%dm[ %[^]]] [%d.%d]]%[^\n]";
          int ansi_color;
          msg[0] = 0;
          num_parsed = sscanf(line.c_str(), log_msg_fmt3, &ansi_color, level, &secs, &nsecs, msg);
          if (num_parsed == 5)
          {
            // Populate new log message
            file[0] = 0;
            function[0] = 0;
            line_num = 0;
            log->file = file;
            log->function = function;
            // log->header.seq = seq;
            // stamp.sec = secs;
            // stamp.nsec = nsecs;
            stamp = rclcpp::Time(secs, nsecs);
            log->stamp = stamp;
            log->level = level_string_to_level_type(std::string(level));
            log->line = line_num;
            log->msg = msg;
          }
          else
          {
            // Example: [ WARN] [1512051107.153917534, 1507066358.521849475]: Offset change exceeds limit! reduce from 0.814476 to 0.500000
            char log_msg_fmt4[] = "\x1b[%dm[ %[^]]] [%d.%d, %d.%d]: %[^\n\x1b]";
            int secs2;
            int nsecs2;
            msg[0] = 0;
            num_parsed = sscanf(line.c_str(), log_msg_fmt4, &ansi_color, level, &secs, &nsecs, &secs2, &nsecs2, msg);
            if (num_parsed == 7)
            {
              // Populate new log message
              file[0] = 0;
              function[0] = 0;
              line_num = 0;
              log->file = file;
              log->function = function;
              // log->header.seq = seq;
              // stamp.sec = secs;
              // stamp.nsec = nsecs;
              stamp = rclcpp::Time(secs, nsecs);
              log->stamp = stamp;
              log->level = level_string_to_level_type(std::string(level));
              log->line = line_num;
              log->msg = msg;
            }
            else // Couldn't parse with known formats
            {
              if (line.length() < MIN_MSG_SIZE)
              {
                return -1;
              }
              log->file = std::string("");
              log->function = std::string("");
              // log->header.seq = seq;
              // stamp.sec = 0;
              // stamp.nsec = 0;
              stamp = rclcpp::Time(0, 0);
              log->stamp = stamp;
              log->level = level_string_to_level_type(std::string("DEBUG"));
              log->line = 0;
              log->msg = line;
              log->name = log->name + "-unparsed";
            }
          }
        }
      }
    }
    return 0;
  }

  rcl_interfaces::msg::Log::_level_type RosoutLogLoader::level_string_to_level_type(const std::string& level_str)
  {
    if (level_str == "FATAL")
    {
      return rcl_interfaces::msg::Log::FATAL;
    }
    if (level_str == "ERROR")
    {
      return rcl_interfaces::msg::Log::ERROR;
    }
    if (level_str == "WARN")
    {
      return rcl_interfaces::msg::Log::WARN;
    }
    if (level_str == "INFO")
    {
      return rcl_interfaces::msg::Log::INFO;
    }
    return rcl_interfaces::msg::Log::DEBUG;
  }


  void RosoutLogLoader::promptForLogFile()
  {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                                                    tr("Open ROS Log File"),
                                                    QDir::homePath(),
                                                    tr("Log Files (*.log)"));

    if (filename != nullptr)
    {
      loadRosLog(filename);
    }
  }

  void RosoutLogLoader::promptForLogDirectory()
  {
    QString dirname = QFileDialog::getExistingDirectory(nullptr,
                                                    tr("Open directory containing log files"),
                                                    QDir::homePath());

    if (dirname != nullptr)
    {
      loadRosLogDirectory(dirname);
    }
  }


}
