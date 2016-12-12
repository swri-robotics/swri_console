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

#include <swri_console/node_click_handler.h>
#include <swri_console/node_list_model.h>

#include <ros/ros.h>
#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

#include <QContextMenuEvent>
#include <QListView>
#include <QMenu>
#include <QTextStream>

namespace swri_console
{
  bool NodeClickHandler::eventFilter(QObject* obj, QEvent* event)
  {
    QContextMenuEvent* context_event;
    switch (event->type()) {
      case QEvent::ContextMenu:
        context_event = static_cast<QContextMenuEvent*>(event);
        break;
      default:
        // Pass through all other events
        return QObject::eventFilter(obj, event);
    }

    // First, make sure we clicked on the list and have an item in the list
    // under the mouse cursor.
    QListView* list = static_cast<QListView*>(obj);
    if (list == NULL) {
      return false;
    }

    QModelIndex index = list->indexAt(context_event->pos());
    if (!index.isValid()) {
      return false;
    }

    // Now get the node name that was clicked on and make a service call to
    // get all of the loggers registered for that node.
    NodeListModel* model = static_cast<NodeListModel*>(list->model());
    node_name_ = model->nodeName(index);

    std::string service_name = node_name_ + "/get_loggers";
    ros::ServiceClient client = nh_.serviceClient<roscpp::GetLoggers>(service_name);
    if (!client.waitForExistence(ros::Duration(2.0)))
    {
      ROS_WARN("Timed out while waiting for service at %s.", service_name.c_str());
      return false;
    }

    roscpp::GetLoggers srv;

    QMenu menu(list);
    QAction* label = menu.addAction(QString::fromStdString(node_name_ + " loggers:"));
    label->setDisabled(true);

    ROS_DEBUG("Getting loggers for %s...", node_name_.c_str());
    if (client.call(srv))
    {
      roscpp::Logger logger;
      Q_FOREACH(logger, srv.response.loggers)
      {
        ROS_DEBUG("Log level for %s is %s", logger.name.c_str(), logger.level.c_str());
        QString action_label;
        QString logger_name = QString::fromStdString(logger.name);
        QTextStream stream(&action_label);
        stream << logger.name.c_str() << " (" << QString::fromStdString(logger.level).toUpper() << ")";

        QMenu* nodeMenu = new QMenu(action_label);
        menu.addMenu(nodeMenu);

        QAction* action = nodeMenu->addAction("DEBUG", this, SLOT(logLevelClicked()));
        action->setData(logger_name);
        action = nodeMenu->addAction("INFO", this, SLOT(logLevelClicked()));
        action->setData(logger_name);
        action = nodeMenu->addAction("WARN", this, SLOT(logLevelClicked()));
        action->setData(logger_name);
        action = nodeMenu->addAction("ERROR", this, SLOT(logLevelClicked()));
        action->setData(logger_name);
        action = nodeMenu->addAction("FATAL", this, SLOT(logLevelClicked()));
        action->setData(logger_name);
      }
    }
    else
    {
      ROS_WARN("Service call to get loggers failed.");
    }

    menu.exec(context_event->globalPos());

    return false;
  }

  void NodeClickHandler::logLevelClicked()
  {
    QAction* action = static_cast<QAction*>(sender());

    std::string logger = action->data().toString().toStdString();
    std::string level = action->text().toStdString();
    ROS_DEBUG("Setting log level for %s/%s to %s", node_name_.c_str(), logger.c_str(), level.c_str());

    std::string service_name = node_name_ + "/set_logger_level";

    ros::ServiceClient client = nh_.serviceClient<roscpp::SetLoggerLevel>(service_name);
    if (!client.waitForExistence(ros::Duration(2.0)))
    {
      ROS_WARN("Timed out while waiting for service at %s.", service_name.c_str());
      return;
    }
    roscpp::SetLoggerLevel srv;
    srv.request.level = level;
    srv.request.logger = logger;
    if (client.call(srv))
    {
      ROS_DEBUG("Set logger level.");
    }
    else
    {
      ROS_WARN("Service call to %s failed.", service_name.c_str());
    }
  }
}

