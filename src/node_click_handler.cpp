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

#include <swri_console/node_click_handler.h>
#include <swri_console/node_list_model.h>

#include <algorithm>
#include <chrono>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#ifdef SWRI_CONSOLE_HAS_LOGGER_SERVICES
#include <rcl_interfaces/msg/logger_level.hpp>
#include <rcl_interfaces/srv/get_logger_levels.hpp>
#include <rcl_interfaces/srv/set_logger_levels.hpp>
#endif

#include <QColorDialog>
#include <QMessageBox>

namespace swri_console
{
  const std::string NodeClickHandler::GET_LOGGER_LEVELS_SVC = "/get_logger_levels";
  const std::string NodeClickHandler::SET_LOGGER_LEVELS_SVC = "/set_logger_levels";

#ifdef SWRI_CONSOLE_HAS_LOGGER_SERVICES
  namespace
  {
    using LoggerLevel = rcl_interfaces::msg::LoggerLevel;

    /**
     * Normally these calls should return very quickly, but we don't want the GUI
     * to hang if a node is stuck, so everything gets a timeout.  The value is
     * pretty arbitrary, but we also want to give enough time for this to still
     * respond over a slow network link, so it shouldn't be *too* small.
     */
    constexpr std::chrono::seconds SERVICE_TIMEOUT{2};

    const std::pair<const char*, uint8_t> LOG_LEVELS[] = {
      {"DEBUG", LoggerLevel::LOG_LEVEL_DEBUG},
      {"INFO", LoggerLevel::LOG_LEVEL_INFO},
      {"WARN", LoggerLevel::LOG_LEVEL_WARN},
      {"ERROR", LoggerLevel::LOG_LEVEL_ERROR},
      {"FATAL", LoggerLevel::LOG_LEVEL_FATAL},
    };

    QString levelName(uint8_t level)
    {
      for (const auto& known : LOG_LEVELS) {
        if (known.second == level) {
          return known.first;
        }
      }
      return "UNKNOWN";
    }

    template<typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
      const rclcpp::Node::SharedPtr& node,
      const std::string& service_name,
      const typename ServiceT::Request::SharedPtr& request)
    {
      auto client = node->create_client<ServiceT>(service_name);
      if (!client->wait_for_service(SERVICE_TIMEOUT)) {
        RCLCPP_WARN(node->get_logger(), "Timed out while waiting for service at %s.",
                    service_name.c_str());
        return nullptr;
      }

      auto future = client->async_send_request(request).future.share();
      if (rclcpp::spin_until_future_complete(node, future, SERVICE_TIMEOUT) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(node->get_logger(), "Service call to %s failed.", service_name.c_str());
        return nullptr;
      }

      return future.get();
    }
  }
#endif

  bool NodeClickHandler::eventFilter(QObject* obj, QEvent* event)
  {
    QContextMenuEvent* context_event;
    QListView* list;

    switch (event->type()) {
      case QEvent::ContextMenu:
        context_event = dynamic_cast<QContextMenuEvent*>(event);
        // First, make sure we clicked on the list and have an item in the list
        // under the mouse cursor.
        list = dynamic_cast<QListView*>(obj);
        if (list == nullptr) {
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

    auto* model = dynamic_cast<NodeListModel*>(list->model());
    if (model == nullptr) {
      return false;
    }

    std::vector<std::string> selected_nodes;
    for (const auto& index : index_list) {
      selected_nodes.push_back(model->nodeName(index));
    }

    QMenu menu(list);

    std::map<QAction*, uint8_t> level_actions;
    addLoggerLevelMenu(&menu, selected_nodes, level_actions);

    QAction* select_color = menu.addAction("Select Color...");
    QAction* clear_color = menu.addAction("Clear Color");

    QAction* chosen = menu.exec(event->globalPos());
    if (chosen == select_color) {
      QColor color = QColorDialog::getColor(Qt::white, list, "Select Node Color");
      if (color.isValid()) {
        for (const auto& node : selected_nodes) {
          Q_EMIT nodeColorSelected(node, color);
        }
      }
    } else if (chosen == clear_color) {
      for (const auto& node : selected_nodes) {
        Q_EMIT nodeColorCleared(node);
      }
    } else {
      auto level = level_actions.find(chosen);
      if (level != level_actions.end()) {
        setLoggerLevels(level->second, list);
      }
    }

    return false;
  }

  void NodeClickHandler::addLoggerLevelMenu(QMenu* menu,
                                            const std::vector<std::string>& loggers,
                                            std::map<QAction*, uint8_t>& level_actions)
  {
    logger_targets_.clear();

#ifndef SWRI_CONSOLE_HAS_LOGGER_SERVICES
    // Distros older than Iron have no logger level services, so there's nothing
    // we could offer here.
    (void)menu;
    (void)loggers;
    (void)level_actions;
#else
    if (!initNode()) {
      return;
    }

    for (const auto& logger : loggers) {
      const std::string node = ownerNode(logger);
      if (!node.empty() && serviceAvailable(node + SET_LOGGER_LEVELS_SVC)) {
        logger_targets_.emplace_back(node, logger);
      }
    }

    if (logger_targets_.empty()) {
      return;
    }

    // Only look up the current level when a single logger is selected; with
    // several of them there's no one level to show.
    QString title = "Set Log Level";
    uint8_t current_level = LoggerLevel::LOG_LEVEL_UNKNOWN;
    if (logger_targets_.size() == 1) {
      current_level = loggerLevel(logger_targets_.front().second);
      if (current_level != LoggerLevel::LOG_LEVEL_UNKNOWN) {
        title += " (" + levelName(current_level) + ")";
      }
    }

    QMenu* submenu = menu->addMenu(title);
    for (const auto& level : LOG_LEVELS) {
      QAction* action = submenu->addAction(level.first);
      action->setCheckable(true);
      action->setChecked(level.second == current_level);
      level_actions[action] = level.second;
    }
    menu->addSeparator();
#endif
  }

  uint8_t NodeClickHandler::loggerLevel(const std::string& logger_name)
  {
#ifndef SWRI_CONSOLE_HAS_LOGGER_SERVICES
    (void)logger_name;
    return 0;
#else
    const std::string node = ownerNode(logger_name);
    const std::string service_name = node + GET_LOGGER_LEVELS_SVC;
    if (node.empty() || !serviceAvailable(service_name)) {
      return LoggerLevel::LOG_LEVEL_UNKNOWN;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
    request->names.push_back(logger_name);

    auto response = callService<rcl_interfaces::srv::GetLoggerLevels>(nh_, service_name, request);
    if (!response || response->levels.empty()) {
      return LoggerLevel::LOG_LEVEL_UNKNOWN;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "Log level for %s is %u",
                 logger_name.c_str(), response->levels.front().level);
    return static_cast<uint8_t>(response->levels.front().level);
#endif
  }

  void NodeClickHandler::setLoggerLevels(uint8_t level, QWidget* parent)
  {
#ifndef SWRI_CONSOLE_HAS_LOGGER_SERVICES
    (void)level;
    (void)parent;
#else
    if (!initNode()) {
      return;
    }

    // Several of the selected loggers may live in the same node, so batch them
    // into a single request per node.
    std::map<std::string, std::vector<std::string>> loggers_by_node;
    for (const auto& target : logger_targets_) {
      loggers_by_node[target.first].push_back(target.second);
    }

    QStringList failures;
    for (const auto& node : loggers_by_node) {
      auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
      for (const auto& logger_name : node.second) {
        RCLCPP_DEBUG(nh_->get_logger(), "Setting log level for %s to %s",
                     logger_name.c_str(), qPrintable(levelName(level)));
        LoggerLevel logger_level;
        logger_level.name = logger_name;
        logger_level.level = level;
        request->levels.push_back(logger_level);
      }

      const std::string service_name = node.first + SET_LOGGER_LEVELS_SVC;
      auto response = callService<rcl_interfaces::srv::SetLoggerLevels>(nh_, service_name, request);
      if (!response || response->results.size() != node.second.size()) {
        failures << QString("%1: service call to %2 failed")
                      .arg(QString::fromStdString(node.first),
                           QString::fromStdString(service_name));
        continue;
      }

      for (size_t i = 0; i < response->results.size(); i++) {
        if (!response->results[i].successful) {
          failures << QString("%1: %2")
                        .arg(QString::fromStdString(node.second[i]),
                             QString::fromStdString(response->results[i].reason));
        }
      }
    }

    if (!failures.isEmpty()) {
      RCLCPP_WARN(nh_->get_logger(), "Failed to set log levels: %s",
                  qPrintable(failures.join("; ")));
      QMessageBox::warning(parent, "Error Setting Log Level",
                           "Failed to set the log level:\n" + failures.join("\n"));
    }
#endif
  }

  bool NodeClickHandler::initNode()
  {
    if (!rclcpp::ok()) {
      // Drop any node left over from a previous ROS session; it can't be used
      // again after a shutdown.
      nh_.reset();
      return false;
    }

    if (!nh_) {
      // Same anonymous naming trick RosThread uses, since we need a name that
      // won't collide with anything else on the graph.
      std::stringstream name;
      name << "swri_console_logger_client_" << rclcpp::Clock().now().nanoseconds();
      nh_ = rclcpp::Node::make_shared(name.str());
    }

    return true;
  }

  std::string NodeClickHandler::ownerNode(const std::string& logger_name)
  {
    if (!initNode()) {
      return "";
    }

    // Node loggers are named after the node's fully qualified name with the
    // separators swapped, so turn that back into something we can match against
    // the graph.
    std::string path = logger_name;
    std::replace(path.begin(), path.end(), '.', '/');
    if (path.empty() || path.front() != '/') {
      path.insert(path.begin(), '/');
    }

    // The logger may be the node's own logger ("/ns/node") or a child logger of
    // it ("/ns/node/sublogger"), so take the longest node name that prefixes it.
    std::string owner;
    for (const auto& node : nh_->get_node_names()) {
      const bool is_match = path == node ||
        (path.size() > node.size() &&
         path.compare(0, node.size(), node) == 0 &&
         path[node.size()] == '/');
      if (is_match && node.size() > owner.size()) {
        owner = node;
      }
    }

    return owner;
  }

  bool NodeClickHandler::serviceAvailable(const std::string& service_name)
  {
    if (!initNode()) {
      return false;
    }

    const auto services = nh_->get_service_names_and_types();
    return services.find(service_name) != services.end();
  }
}
