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

#ifndef SWRI_CONSOLE_UNIX_SIGNAL_HANDLER_H
#define SWRI_CONSOLE_UNIX_SIGNAL_HANDLER_H

#include <QObject>

class QSocketNotifier;

namespace swri_console
{
  /**
   * Bridges POSIX SIGINT/SIGTERM delivery into the Qt event loop.
   *
   * rclcpp::init() installs its own SIGINT handler that shuts down the ROS
   * context, but has no way to stop QApplication::exec(), so the GUI event
   * loop (and thus the process) never exits on Ctrl+C. This class uses the
   * standard Qt self-pipe/socketpair pattern: the async-signal-safe handler
   * only writes a byte to a socket, and a QSocketNotifier on the Qt thread
   * reacts to that by calling QCoreApplication::quit(), which lets the
   * process shut down cleanly instead of relying on an external supervisor
   * to escalate to SIGTERM/SIGKILL after a timeout.
   */
  class UnixSignalHandler : public QObject
  {
    Q_OBJECT
  public:
    explicit UnixSignalHandler(QObject* parent = nullptr);

    // Installs the SIGINT/SIGTERM handlers. Must be called after the
    // QApplication has been constructed.
    void setup();

    // Async-signal-safe handlers registered with sigaction().
    static void intSignalHandler(int unused);
    static void termSignalHandler(int unused);

  private Q_SLOTS:
    void handleSigInt();
    void handleSigTerm();

  private:
    static int sigintFd[2];
    static int sigtermFd[2];

    QSocketNotifier* sn_int_;
    QSocketNotifier* sn_term_;
  };
}

#endif //SWRI_CONSOLE_UNIX_SIGNAL_HANDLER_H
