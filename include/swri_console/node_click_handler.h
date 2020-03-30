// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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

#ifndef SWRI_CONSOLE_NODE_CLICK_HANDLER_H
#define SWRI_CONSOLE_NODE_CLICK_HANDLER_H

#include <vector>

#include <boost/thread.hpp>

#include <QContextMenuEvent>
#include <QObject>
#include <QEvent>
#include <QFuture>
#include <QListView>
#include <QMenu>

#include <rclcpp/rclcpp.hpp>

namespace swri_console
{
  class NodeClickHandler : public QObject
  {
    Q_OBJECT

  public Q_SLOTS:
    void logLevelClicked();

  protected:
    bool eventFilter(QObject* obj, QEvent* event);

  private:
    bool showContextMenu(QListView* list, QContextMenuEvent* event);
    QMenu* createMenu(const QString& logger_name, const QString& current_level);

    std::shared_ptr<rclcpp::Node> nh_;
    std::string node_name_;
    std::vector<std::string> all_loggers_;

    static const std::string ALL_LOGGERS;
    static const std::string GET_LOGGERS_SVC;
    static const std::string SET_LOGGER_LEVEL_SVC;
  };
}

#endif //SWRI_CONSOLE_NODE_CLICK_HANDLER_H
