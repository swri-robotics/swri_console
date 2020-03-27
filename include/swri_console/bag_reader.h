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

#ifndef SWRI_CONSOLE_BAG_READER_H
#define SWRI_CONSOLE_BAG_READER_H

#include <QObject>
#include <QString>
#include <QMetaType>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

namespace swri_console
{
  class BagReader : public QObject
  {
    Q_OBJECT
  public:
    /**
     * Reads a bag file at the specified path.  Any log messages that were broadcast on the
     * /rosout topic will be loaded and displayed.
     * @param[in] filename The name of the bag file to load.
     */
    void readBagFile(const QString& filename);

  public Q_SLOTS:
    /**
     * Displays a file dialog that prompts the user to pick a bag file.  After picking a bag
     * file, log messages in the bag will be read and displayed in the log console.
     */
    void promptForBagFile();

  Q_SIGNALS:

    /**
     * Emitted every time a log message is received.  This will likely be emitted several times
     * per bag file; finishedReading will be emitted when we're done.
     */
    void logReceived(const rcl_interfaces::msg::Log::ConstSharedPtr);

    /**
     * Emitted after we're completely done reading the bag file.
     */
    void finishedReading();
  };
}

#endif //SWRI_CONSOLE_BAG_READER_H
