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

#ifndef ROSOUT_LOG_LOADER_H
#define ROSOUT_LOG_LOADER_H

#include <QObject>
#include <QString>
#include <QMetaType>

#include <rosgraph_msgs/Log.h>

namespace swri_console
{
  class RosoutLogLoader : public QObject
  {
    Q_OBJECT
  public:
    void loadRosLog(const QString& filename);
    void loadRosLogDirectory(const QString& logdirectory_name);

  public Q_SLOTS:
    void promptForLogFile();
    void promptForLogDirectory();

  Q_SIGNALS:
    /**
     * Emitted every time a log message is received.  This will likely be emitted several times
     * per bag file; finishedReading will be emitted when we're done.
     */
    void logReceived(const rosgraph_msgs::LogConstPtr& msg);

    /**
     * Emitted after we're completely done reading the bag file.
     */
    void finishedReading();

  private:
    int parseLine(std::string line, int seq, rosgraph_msgs::Log* log);
    rosgraph_msgs::Log::_level_type level_string_to_level_type(std::string level_str);
  };
}

#endif // ROSOUT_LOG_LOADER_H
