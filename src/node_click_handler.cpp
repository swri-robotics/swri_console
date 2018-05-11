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

#include <QMessageBox>
#include <QTextStream>

namespace swri_console
{
  const std::string NodeClickHandler::ALL_LOGGERS = "All Loggers";
  const std::string NodeClickHandler::GET_LOGGERS_SVC = "/get_loggers";
  const std::string NodeClickHandler::SET_LOGGER_LEVEL_SVC = "/set_logger_level";

  bool NodeClickHandler::eventFilter(QObject* obj, QEvent* event)
  {
    QContextMenuEvent* context_event;
    QListView* list;

    switch (event->type()) {
      case QEvent::ContextMenu:
        context_event = static_cast<QContextMenuEvent*>(event);
        // First, make sure we clicked on the list and have an item in the list
        // under the mouse cursor.
        list = static_cast<QListView*>(obj);
        if (list == NULL) {
          return false;
        }

        return showContextMenu(list, context_event);
      default:
        // Pass through all other events
        return QObject::eventFilter(obj, event);
    }
  }

  bool NodeClickHandler::showContextMenu(QListView* list, QContextMenuEvent* event)
  {
    QModelIndexList index_list = list->selectionModel()->selectedIndexes();
    if (index_list.isEmpty()) {
      return false;
    }

    // Now get the node name that was clicked on and make a service call to
    // get all of the loggers registered for that node.
    NodeListModel* model = static_cast<NodeListModel*>(list->model());
    node_name_ = model->nodeName(index_list.first());

    std::string service_name = node_name_ + GET_LOGGERS_SVC;
    ros::ServiceClient client = nh_.serviceClient<roscpp::GetLoggers>(service_name);
    /**
     * Normally this call should return very quickly, but we don't want the GUI to
     * hang if the roscore is stuck, so add a timeout.
     * The value is pretty arbitrary, but we also want to give enough time for this
     * to still respond over a slow network link, so it shouldn't be *too* small.
     */
    if (!client.waitForExistence(ros::Duration(2.0))) {
      ROS_WARN("Timed out while waiting for service at %s.", service_name.c_str());
      QMessageBox::warning(list, "Error Getting Loggers", "Timed out waiting for get_loggers service.");
      return false;
    }

    roscpp::GetLoggers srv;

    QMenu menu(list);
    QAction* label = menu.addAction(QString::fromStdString(node_name_ + " loggers:"));
    label->setDisabled(true);

    ROS_DEBUG("Getting loggers for %s...", node_name_.c_str());
    if (callService(client, srv)) {
      all_loggers_.clear();
      menu.addMenu(createMenu(QString::fromStdString(ALL_LOGGERS), ""));
      Q_FOREACH(const roscpp::Logger& logger, srv.response.loggers) {
        ROS_DEBUG("Log level for %s is %s", logger.name.c_str(), logger.level.c_str());
        all_loggers_.push_back(logger.name);
        QString logger_name = QString::fromStdString(logger.name);

        menu.addMenu(createMenu(logger_name, QString::fromStdString(logger.level)));
      }
    }
    else {
      std::string error = "Service call to get_loggers failed.";
      ROS_WARN("%s", error.c_str());
      QMessageBox::warning(list, "Service Call Failed", error.c_str());
    }

    menu.exec(event->globalPos());

    return false;
  }

  QMenu* NodeClickHandler::createMenu(const QString& logger_name, const QString& current_level)
  {
    QString action_label;
    QTextStream stream(&action_label);
    stream << logger_name;
    if (!current_level.isEmpty()) {
      stream << " (" << current_level.toUpper() << ")";
    }

    QMenu* nodeMenu = new QMenu(action_label);

    const QString levels[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

    for (int i = 0; i < 5; i++) {
      QAction* action = nodeMenu->addAction(levels[i], this, SLOT(logLevelClicked()));
      action->setData(logger_name);
    }

    return nodeMenu;
  }

  void NodeClickHandler::logLevelClicked()
  {
    QAction* action = static_cast<QAction*>(sender());

    std::string logger = action->data().toString().toStdString();
    std::string level = action->text().toStdString();
    ROS_DEBUG("Setting log level for %s/%s to %s", node_name_.c_str(), logger.c_str(), level.c_str());

    std::string service_name = node_name_ + SET_LOGGER_LEVEL_SVC;

    ros::ServiceClient client = nh_.serviceClient<roscpp::SetLoggerLevel>(service_name);
    if (!client.waitForExistence(ros::Duration(2.0))) {
      ROS_WARN("Timed out while waiting for service at %s.", service_name.c_str());
      QMessageBox::warning(NULL, "Error Getting Loggers", "Timed out waiting for set_logger_level service.");
      return;
    }

    std::vector<std::string> target_loggers;
    if (logger == ALL_LOGGERS) {
      target_loggers = all_loggers_;
    }
    else {
      target_loggers.push_back(logger);
    }

    Q_FOREACH (const std::string& logger_name, target_loggers) {
      roscpp::SetLoggerLevel srv;
      srv.request.level = level;
      srv.request.logger = logger_name;
      if (callService(client, srv)) {
        ROS_DEBUG("Set logger level.");
      }
      else {
        ROS_WARN("Service call to %s failed.", service_name.c_str());
        QMessageBox::warning(NULL, "Error Setting Log Level", "Failed to set logger level.");
      }
    }
  }
}

