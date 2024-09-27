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

#include <cstdio>
#include <algorithm>
#include <iterator>
#include <iomanip>
#include <chrono>
#include <ctime>

#include <rclcpp/rclcpp.hpp>

#include <swri_console/log_database_proxy_model.h>
#include <swri_console/log_database.h>
#include <swri_console/settings_keys.h>

#include <QColor>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QTimer>
#include <QSettings>
#include <QtGlobal>

namespace swri_console
{
LogDatabaseProxyModel::LogDatabaseProxyModel(LogDatabase *db)
  : QAbstractListModel()
  , severity_mask_(0)
  , colorize_logs_(true)
  , display_time_(true)
  , display_absolute_time_(false)
  , human_readable_time_(false)
  , display_logger_(true)
  , display_function_(true)
  , use_regular_expressions_(false)
  , latest_log_index_(0)
  , earliest_log_index_(0)
  , debug_color_(Qt::gray)
  , info_color_(Qt::black)
  , warn_color_(QColor(255,127,0))
  , error_color_(Qt::red)
  , fatal_color_(Qt::magenta)
  , db_(db)
  , failedSearchText_("")
  , failedSearchIndex_(0)
{
  QObject::connect(db_, SIGNAL(databaseCleared()),
                   this, SLOT(handleDatabaseCleared()));
  QObject::connect(db_, SIGNAL(messagesAdded()),
                   this, SLOT(processNewMessages()));

  QObject::connect(db_, SIGNAL(minTimeUpdated()),
                   this, SLOT(minTimeUpdated()));
}

void LogDatabaseProxyModel::setNodeFilter(const std::set<std::string> &names)
{
  names_ = names;
  reset();
}

void LogDatabaseProxyModel::setSeverityFilter(uint8_t severity_mask)
{
  severity_mask_ = severity_mask;
  reset();
}

void LogDatabaseProxyModel::setAbsoluteTime(bool absolute)
{
  if (absolute == display_absolute_time_) {
    return;
  }

  display_absolute_time_ = absolute;

  QSettings settings;
  settings.setValue(SettingsKeys::ABSOLUTE_TIMESTAMPS, display_absolute_time_);

  if (display_time_ && !msg_mapping_.empty()) {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }
}

void LogDatabaseProxyModel::setHumanReadableTime(bool human_readable_time)
{
  if (human_readable_time == human_readable_time_)
  {
    return;
  }

  human_readable_time_ = human_readable_time;

  QSettings settings;
  settings.setValue(SettingsKeys::HUMAN_READABLE_TIME, human_readable_time_);

  if (display_time_ && msg_mapping_.size())
  {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }
}

void LogDatabaseProxyModel::setColorizeLogs(bool colorize_logs)
{
  if (colorize_logs == colorize_logs_) {
    return;
  }

  colorize_logs_ = colorize_logs;
  QSettings settings;
  settings.setValue(SettingsKeys::COLORIZE_LOGS, colorize_logs_);

  if (!msg_mapping_.empty()) {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }
}

void LogDatabaseProxyModel::setDisplayTime(bool display)
{
  if (display == display_time_) {
    return;
  }

  display_time_ = display;

  QSettings settings;
  settings.setValue(SettingsKeys::DISPLAY_TIMESTAMPS, display_time_);

  if (!msg_mapping_.empty()) {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }
}

void LogDatabaseProxyModel::setDisplayLogger(bool logger_name)
{
  if (logger_name == display_logger_) {
    return;
  }

  display_logger_ = logger_name;

  QSettings settings;
  settings.setValue(SettingsKeys::DISPLAY_LOGGER, display_logger_);

  if (!msg_mapping_.empty()) {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }
}

void LogDatabaseProxyModel::setDisplayFunction(bool function_name)
{
  if (function_name == display_function_) {
    return;
  }

  display_function_ = function_name;

  QSettings settings;
  settings.setValue(SettingsKeys::DISPLAY_FUNCTION, display_function_);

  if (!msg_mapping_.empty()) {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }
}

void LogDatabaseProxyModel::setUseRegularExpressions(bool useRegexps)
{
  if (useRegexps == use_regular_expressions_) {
    return;
  }

  use_regular_expressions_ = useRegexps;
  QSettings settings;
  settings.setValue(SettingsKeys::USE_REGEXPS, useRegexps);
  reset();
}

void LogDatabaseProxyModel::setIncludeFilters(
  const QStringList &list)
{
  include_strings_ = list;
  // The text and regexp filters are always updated at the same time, so this
  // value will be saved by setIncludeRegexpPattern.
  reset();
}

void LogDatabaseProxyModel::setExcludeFilters(
  const QStringList &list)
{
  exclude_strings_ = list;
  // The text and regexp filters are always updated at the same time, so this
  // value will be saved by setExcludeRegexpPattern.
  reset();
}


void LogDatabaseProxyModel::setIncludeRegexpPattern(const QString& pattern)
{
  include_regexp_.setPattern(pattern);
  QSettings settings;
  settings.setValue(SettingsKeys::INCLUDE_FILTER, pattern);
  reset();
}

void LogDatabaseProxyModel::setExcludeRegexpPattern(const QString& pattern)
{
  exclude_regexp_.setPattern(pattern);
  QSettings settings;
  settings.setValue(SettingsKeys::EXCLUDE_FILTER, pattern);
  reset();
}

void LogDatabaseProxyModel::setDebugColor(const QColor& debug_color)
{
  debug_color_ = debug_color;
  QSettings settings;
  settings.setValue(SettingsKeys::DEBUG_COLOR, debug_color);
  reset();
}

void LogDatabaseProxyModel::setInfoColor(const QColor& info_color)
{
  info_color_ = info_color;
  QSettings settings;
  settings.setValue(SettingsKeys::INFO_COLOR, info_color);
  reset();
}

void LogDatabaseProxyModel::setWarnColor(const QColor& warn_color)
{
  warn_color_ = warn_color;
  QSettings settings;
  settings.setValue(SettingsKeys::WARN_COLOR, warn_color);
  reset();
}

void LogDatabaseProxyModel::setErrorColor(const QColor& error_color)
{
  error_color_ = error_color;
  QSettings settings;
  settings.setValue(SettingsKeys::ERROR_COLOR, error_color);
  reset();
}

void LogDatabaseProxyModel::setFatalColor(const QColor& fatal_color)
{
  fatal_color_ = fatal_color;
  QSettings settings;
  settings.setValue(SettingsKeys::FATAL_COLOR, fatal_color);
  reset();
}

int LogDatabaseProxyModel::rowCount(const QModelIndex &parent) const
{
  if (parent.isValid()) {
    return 0;
  }

  return msg_mapping_.size();
}


bool LogDatabaseProxyModel::isIncludeValid() const
{
  if (use_regular_expressions_ && !include_regexp_.isValid()) {
    return false;
  }
  return true;
}

bool LogDatabaseProxyModel::isExcludeValid() const
{
  if (use_regular_expressions_ && !exclude_regexp_.isValid()) {
    return false;
  }
  return true;
}

// Locates the next index based on search criteria, VCM 25 April 2017
// searchText_ - string from searchText, all upper case and trimmed spaces
// index - currently selected item in messageList
// increment - +1 = next||search(i.e. down), -1 = prev (i.e. up)
int LogDatabaseProxyModel::getItemIndex(const QString& searchText, int index, int increment)
{
  int searchNotFound = -1;  // indicates search not found
  int counter=0;  // used to stop loop once full list has been searched
  bool partialSearch = false;  // tells main loop to run a partial search, triggered by prior failed search
  if(searchText.isEmpty() || msg_mapping_.empty())  // skip search for 1)empty string 2)empty set
  {
    clearSearchFailure();  // reset failed search variables
    return searchNotFound;
  }

  // round corners for searches
  if(index<0)  // if index < 0, set to size()-1;
  {
    index = static_cast<int>(msg_mapping_.size()) - 1;
  }
  else if(index >= msg_mapping_.size())  // if index >size(), set to 0;
  {
    index = 0;
  }

  // trigger partial search if:
  //   searchText_ conaints prior failed text
  //   prior failed text is not empty
  //   failed index is not 0
  //   failed search index isn't greater than current index, this could happen through user
  //     interface message selection. Software should clear the variables when UI is adjusted.
  if(searchText.contains(failedSearchText_) && failedSearchText_ != "" && failedSearchIndex_ !=0 && failedSearchIndex_ <= msg_mapping_.size() )
  {
    partialSearch = true;
    index = failedSearchIndex_-1;
    counter = failedSearchIndex_;
  }
  int i;
  for(i=0; i<msg_mapping_.size();i++)  // loop through all messages until end or match is found
  {
    const LineMap line_idx = msg_mapping_[index];
    const LogEntry &item = db_->log()[line_idx.log_index];
    QString tempString = item.text.join("|");  // concatenate strings
    if(tempString.toUpper().contains(searchText))  // search match found
    {
      clearSearchFailure();  // reset failed search variables
      return index;  // match found, return location and exit loop
    }
    counter++;  // used to track total search length
    if(counter>=msg_mapping_.size())  // exit if all messages have been scanned
    {
      if((!partialSearch)||(failedSearchText_ == ""))  // store failed text if one isn't already stored
      {
        failedSearchText_ = searchText;
      }
      failedSearchIndex_ = msg_mapping_.size();
      return searchNotFound;  // match not found, return -1 and exit loop
    }
    // increment (next/search) or decrement (prev) index then address corner rounding
    index = index + increment;
    if(index<0)  // less than 0 set to max
    {
      index = static_cast<int>(msg_mapping_.size() - 1);
    }
    else if(index>=msg_mapping_.size())  // greater than max, set to 0
    {
      index = 0;
    }
  }

  return -1;
}

void LogDatabaseProxyModel::clearSearchFailure()
{
  // reset failed search variables, VCM 27 April 2017
  failedSearchIndex_ = 0;
  failedSearchText_ = "";
}

QVariant LogDatabaseProxyModel::data(
  const QModelIndex &index, int role) const
{
  switch (role)
  {
    // Currently we're only returning data for these roles, so return immediately
    // if we're being queried for anything else.
    case Qt::DisplayRole:
    case Qt::ToolTipRole:
    case ExtendedLogRole:
      break;
    case Qt::ForegroundRole:
      if (colorize_logs_) {
        break;
      }
    default:
      return QVariant();
  }

  if (index.parent().isValid() &&
      static_cast<size_t>(index.row()) >= msg_mapping_.size()) {
    return QVariant();
  }

  const LineMap line_idx = msg_mapping_[index.row()];
  const LogEntry &item = db_->log()[line_idx.log_index];

  if (role == Qt::DisplayRole) {
    char level = '?';
    if (item.getLogLvl() == rcl_interfaces::msg::Log::DEBUG) {
      level = 'D';
    } else if (item.getLogLvl() == rcl_interfaces::msg::Log::INFO) {
      level = 'I';
    } else if (item.getLogLvl() == rcl_interfaces::msg::Log::WARN) {
      level = 'W';
    } else if (item.getLogLvl() == rcl_interfaces::msg::Log::ERROR) {
      level = 'E';
    } else if (item.getLogLvl() == rcl_interfaces::msg::Log::FATAL) {
      level = 'F';
    }

    char stamp[128];
    if (display_absolute_time_) {
      if (human_readable_time_) {
        char date_str[std::size("yyyy-mm-dd hh:mm:ss")];
        const time_t time = static_cast<time_t>(item.stamp.seconds());
        int32_t milliseconds = static_cast<int>(1000.0 * (item.stamp.seconds() - std::floor(item.stamp.seconds())));
        std::strftime(std::data(date_str),
          std::size(date_str),
          "%F %T",
          std::localtime(&time));
        snprintf(stamp,
          sizeof(stamp),
          "%s:%03d",
          date_str,
          milliseconds);
      } else {
        snprintf(stamp,
          sizeof(stamp),
          "%f",
          item.stamp.seconds());
      }
    } else {
      rclcpp::Duration t = item.stamp - db_->minTime();

      int32_t secs = t.seconds();
      int hours = secs / 60 / 60;
      int minutes = (secs / 60) % 60;
      int seconds = (secs % 60);
      int milliseconds = static_cast<int>(1000.0 * (t.seconds() - static_cast<double>(secs)));
      
      snprintf(stamp, sizeof(stamp),
               "%d:%02d:%02d:%03d",
               hours, minutes, seconds, milliseconds);
    }

    char id[256];
    if (display_logger_ && display_function_) {
      snprintf(id, sizeof(id), "%s::%s", item.node.c_str(), item.function.c_str());
    } else if (display_logger_ && !display_function_) {
      snprintf(id, sizeof(id), "%s", item.node.c_str());
    } else if (!display_logger_ && display_function_) {
      snprintf(id, sizeof(id), "::%s", item.function.c_str());
    }

    bool display_id = display_logger_ || display_function_;

    char header[1024];
    if (display_time_ && display_id) {
      snprintf(header, sizeof(header), "%c %s [%s] ", level, stamp, id);
    } else if (display_time_) {
      snprintf(header, sizeof(header), "%c %s [] ", level, stamp);
    } else if (display_id) {
      snprintf(header, sizeof(header), "%c [%s] ", level, id);
    } else {
      snprintf(header, sizeof(header), "%c [] ", level);
    }

    // For multiline messages, we only want to display the header for
    // the first line.  For the subsequent lines, we generate a header
    // and then fill it with blank lines so that the messages are
    // aligned properly (assuming monospaced font).  
    if (line_idx.line_index != 0) {
      size_t len = strnlen(header, sizeof(header));
      for (size_t i = 0; i < len; i++) {
        header[i] = ' ';
      }
    }
    
    return QVariant(QString(header) + item.text[line_idx.line_index]);
  }
  else if (role == Qt::ForegroundRole && colorize_logs_) {
    switch (item.getLogLvl()) {
      case rcl_interfaces::msg::Log::DEBUG:
        return QVariant(debug_color_);
      case rcl_interfaces::msg::Log::INFO:
        return QVariant(info_color_);
      case rcl_interfaces::msg::Log::WARN:
        return QVariant(warn_color_);
      case rcl_interfaces::msg::Log::ERROR:
        return QVariant(error_color_);
      case rcl_interfaces::msg::Log::FATAL:
        return QVariant(fatal_color_);
      default:
        return QVariant(info_color_);
    }
  }
  else if (role == Qt::ToolTipRole) {
    char buffer[4096];
    snprintf(buffer, sizeof(buffer),
             "<p style='white-space:pre'>"
             "Timestamp: %f\n"
             "Seq: %d\n"
             "Node: %s\n"
             "Function: %s\n"
             "File: %s\n"
             "Line: %d\n"
             "\n",
             item.stamp.seconds(),
             item.seq,
             item.node.c_str(),
             item.function.c_str(),
             item.file.c_str(),
             item.line);
    
    QString text = (QString(buffer) +
                    item.text.join("\n") + 
                    QString("</p>"));
                            
    return QVariant(text);
  } else if (role == LogDatabaseProxyModel::ExtendedLogRole) {
    char buffer[4096];
    snprintf(buffer, sizeof(buffer),
             "Timestamp: %f\n"
             "Node: %s\n"
             "Function: %s\n"
             "File: %s\n"
             "Line: %d\n"
             "Message: ",
             item.stamp.seconds(),
             item.node.c_str(),
             item.function.c_str(),
             item.file.c_str(),
             item.line);
    
    QString text = (QString(buffer) +
                    item.text.join("\n")); 
                            
    return QVariant(text);
  }
      
  return QVariant();
}

void LogDatabaseProxyModel::reset()
{
  beginResetModel();
  msg_mapping_.clear();
  early_mapping_.clear();
  earliest_log_index_ = db_->log().size();
  latest_log_index_ = earliest_log_index_;
  endResetModel();
  scheduleIdleProcessing();
}


void LogDatabaseProxyModel::saveToFile(const QString& filename) const
{
  if (filename.endsWith(".bag", Qt::CaseInsensitive)) {
    QMessageBox::information(nullptr,
                             tr("Bag Files not supported"),
                             tr("Reading and writing bag files is not yet supported in ROS 2."));
    //saveBagFile(filename);
  }
  else {
    saveTextFile(filename);
  }
}

void LogDatabaseProxyModel::saveBagFile(const QString& filename) const
{
  // Set up message serialization
  auto serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  auto initial_capacity = 0u;
  auto ret = rmw_serialized_message_init(
    &serialized_msg,
    initial_capacity,
    &allocator
  );
  auto log_ts = rosidl_typesupport_cpp::get_message_type_support_handle<rcl_interfaces::msg::Log>();
/*
  // rosbag::Bag bag(filename.toStdString().c_str(), rosbag::bagmode::Write);
  rosbag2::Writer bagwriter = rosbag2::Writer();

  // Minimum time value ROS 2 can support
  rclcpp::Time TIME_MIN = rclcpp::Time(std::numeric_limits<rcl_time_point_value_t>::min());

  size_t idx = 0;
  while (idx < msg_mapping_.size()) {
    const LineMap line_map = msg_mapping_[idx];    
    const LogEntry &item = db_->log()[line_map.log_index];
    
    rcl_interfaces::msg::Log log;
    log.file = item.file;
    log.function = item.function;
    // log.header.seq = item.seq;
    if (item.stamp < TIME_MIN) {
      // Note: I think TIME_MIN is the minimum representation of
      // ros::Time, so this branch should be impossible.  Nonetheless,
      // it doesn't hurt.
      log.stamp = rclcpp::Time();
      qWarning("Msg had time (%d); it's less than ros::TIME_MIN, which is invalid. "
               "Writing 'now' instead.",
               item.stamp.seconds());
    } else {
      log.stamp = item.stamp;
    }
    log.level = item.level;
    log.line = item.line;
    log.msg = item.text.join("\n").toStdString();
    log.name = item.node;

    // Serialize for storage
    ret = rmw_serialize(&log, log_ts, &serialized_msg);
    rosbag2_storage::SerializedBagMessage bag_message;
    bag_message.serialized_data = serialized_msg.buffer;

    bag.write("/rosout", log.stamp, log);

    // Advance to the next line with a different log index.
    idx++;
    while (idx < msg_mapping_.size() && msg_mapping_[idx].log_index == line_map.log_index) {
      idx++;
    }
  }
  bag.close();
  */
}

void LogDatabaseProxyModel::saveTextFile(const QString& filename) const
{
  QFile outFile(filename);
  outFile.open(QFile::WriteOnly);
  QTextStream outstream(&outFile);
  for(size_t i = 0; i < msg_mapping_.size(); i++)
  {
    QString line = data(index(i), Qt::DisplayRole).toString();
    outstream << line << '\n';
  }
  outstream.flush();
  outFile.close();
}

void LogDatabaseProxyModel::handleDatabaseCleared()
{
  reset();
  clearSearchFailure();  // reset failed search variables, VCM 26 April 2017
}

void LogDatabaseProxyModel::processNewMessages()
{
  std::deque<LineMap> new_items;
 
  // Process all messages from latest_log_index_ to the end of the
  // log.
  for (;
       latest_log_index_ < db_->log().size();
       latest_log_index_++)
  {
    const LogEntry &item = db_->log()[latest_log_index_];    
    if (!acceptLogEntry(item)) {
      continue;
    }    

    for (int i = 0; i < item.text.size(); i++) {
      new_items.emplace_back(latest_log_index_, i);
    }
  }
  
  if (!new_items.empty()) {
    beginInsertRows(QModelIndex(),
                    msg_mapping_.size(),
                    static_cast<int>(msg_mapping_.size() + new_items.size() - 1));
    msg_mapping_.insert(msg_mapping_.end(),
                        new_items.begin(),
                        new_items.end());
    endInsertRows();

    Q_EMIT messagesAdded();
  }  
}

void LogDatabaseProxyModel::processOldMessages()
{
  // We process old messages in two steps.  First, we process the
  // remaining messages in chunks and store them in the early_mapping_
  // buffer if they pass all the filters.  When the early mapping
  // buffer is large enough (or we have processed everything), then we
  // merge the early_mapping buffer in the main buffer.  This approach
  // allows us to process very large logs without causing major lag
  // for the user.
  
  for (size_t i = 0;
       earliest_log_index_ != 0 && i < 100;
       earliest_log_index_--, i++)
  {
    const LogEntry &item = db_->log()[earliest_log_index_-1];
    if (!acceptLogEntry(item)) {
      continue;
    }

    for (int i = 0; i < item.text.size(); i++) {
      // Note that we have to add the lines backwards to maintain the proper order.
      early_mapping_.push_front(
        LineMap(earliest_log_index_-1, item.text.size()-1-i));
    }
  }
 
  if ((earliest_log_index_ == 0 && !early_mapping_.empty()) ||
      (early_mapping_.size() > 200)) {
    beginInsertRows(QModelIndex(),
                    0,
                    static_cast<int>(early_mapping_.size() - 1));
    msg_mapping_.insert(msg_mapping_.begin(),
                        early_mapping_.begin(),
                        early_mapping_.end());
    early_mapping_.clear();
    endInsertRows();

    Q_EMIT messagesAdded();
  }

  scheduleIdleProcessing();
}

void LogDatabaseProxyModel::scheduleIdleProcessing()
{
  // If we have older logs that still need to be processed, schedule a
  // callback at the next idle time.
  if (earliest_log_index_ > 0) {
    QTimer::singleShot(0, this, SLOT(processOldMessages()));
  }
}

bool LogDatabaseProxyModel::acceptLogEntry(const LogEntry &item)
{
  if (!(item.level_mask & severity_mask_)) {
    return false;
  }
  
  if (names_.count(item.node) == 0) {
    return false;
  }

  if (!testIncludeFilter(item)) {
    return false;
  }

  if (use_regular_expressions_) {
    // For multi-line messages, we join the lines together with a
    // space to make it easy for users to use filters that spread
    // across the new lines.
    
    // Don't let an empty regexp filter out everything
    return exclude_regexp_.isEmpty() || exclude_regexp_.indexIn(item.text.join(" ")) < 0;
  } else {
    for (int i = 0; i < exclude_strings_.size(); i++) {
      if (item.text.join(" ").contains(exclude_strings_[i], Qt::CaseInsensitive)) {
        return false;
      }
    }
  }
  
  return true;
}

// Return true if the item message contains at least one of the
// strings in include_filter_.  Always returns true if there are no
// include strings.
bool LogDatabaseProxyModel::testIncludeFilter(const LogEntry &item)
{
  if (use_regular_expressions_) {
    return include_regexp_.indexIn(item.text.join(" ")) >= 0;
  } else {
    if (include_strings_.empty()) {
      return true;
    }

    for (int i = 0; i < include_strings_.size(); i++) {
      if (item.text.join(" ").contains(include_strings_[i], Qt::CaseInsensitive)) {
        return true;
      }
    }
  }

  return false;
}

void LogDatabaseProxyModel::minTimeUpdated()
{
  if (display_time_ &&
      !display_absolute_time_
      && !msg_mapping_.empty()) {
    Q_EMIT dataChanged(index(0), index(msg_mapping_.size()));
  }  
}
}  // namespace swri_console
