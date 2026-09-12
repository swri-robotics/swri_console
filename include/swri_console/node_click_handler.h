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

#ifndef SWRI_CONSOLE_NODE_CLICK_HANDLER_H
#define SWRI_CONSOLE_NODE_CLICK_HANDLER_H

#include <map>
#include <string>
#include <vector>

#include <QAction>
#include <QColor>
#include <QContextMenuEvent>
#include <QObject>
#include <QEvent>
#include <QListView>
#include <QMenu>

#include <rclcpp/rclcpp.hpp>

namespace swri_console
{
  class NodeClickHandler : public QObject
  {
    Q_OBJECT

  Q_SIGNALS:
    void nodeColorSelected(const std::string& node, const QColor& color);
    void nodeColorCleared(const std::string& node);

  protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

  private:
    bool showContextMenu(QListView* list, QContextMenuEvent* event);

    /**
     * Adds a submenu for setting the log level of the clicked loggers, and fills
     * in level_actions with the level each of its actions would set.  The
     * loggers that the submenu applies to are remembered in logger_targets_.
     *
     * If we can't talk to ROS, if none of the loggers belong to a node that is
     * currently advertising the logger level services, or if we're built
     * against a distro older than Iron (those have no such services), a
     * disabled "Set Log Level" entry is added instead, with a tooltip
     * explaining why.
     */
    void addLoggerLevelMenu(QMenu* menu,
                            const std::vector<std::string>& loggers,
                            std::map<QAction*, uint8_t>& level_actions);
    /**
     * Sets every logger in logger_targets_ to the given level, using one service
     * call per node that owns them.  Warns through parent if a call fails.
     */
    void setLoggerLevels(uint8_t level, QWidget* parent);
    /**
     * Returns the current level of a logger, or LOG_LEVEL_UNKNOWN if it can't be
     * determined.
     */
    uint8_t loggerLevel(const std::string& logger_name);

    /**
     * Lazily creates the private node used to make the logger level service
     * calls.  It is deliberately not the node owned by RosThread, because that
     * one is spun from the ROS thread and these calls are made from the GUI
     * thread.  Returns false if ROS isn't up.
     */
    bool initNode();
    /**
     * Maps a logger name as it appears in /rosout onto the fully qualified name
     * of the node that owns it, e.g. "ns.node.sublogger" -> "/ns/node".  Returns
     * an empty string if no matching node is visible on the graph.
     */
    std::string ownerNode(const std::string& logger_name);
    bool serviceAvailable(const std::string& service_name);

    rclcpp::Node::SharedPtr nh_;
    /// Loggers the level submenu applies to, paired with the node that owns them.
    std::vector<std::pair<std::string, std::string>> logger_targets_;

    static const std::string GET_LOGGER_LEVELS_SVC;
    static const std::string SET_LOGGER_LEVELS_SVC;
  };
}

#endif //SWRI_CONSOLE_NODE_CLICK_HANDLER_H
