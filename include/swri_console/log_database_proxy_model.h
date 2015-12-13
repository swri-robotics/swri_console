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

#ifndef SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_
#define SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_

#include <QAbstractListModel>
#include <QColor>
#include <stdint.h>
#include <set>
#include <string>
#include <deque>
#include <QStringList>
#include <QRegExp>

namespace swri_console
{

class LogDatabase;
class LogEntry;
class LogDatabaseProxyModel : public QAbstractListModel
{
  Q_OBJECT
 
 public:
  enum {
    ExtendedLogRole = Qt::UserRole + 0
  };

  LogDatabaseProxyModel(LogDatabase *db);
  ~LogDatabaseProxyModel();

  void setNodeFilter(const std::set<std::string> &names);
  void setSeverityFilter(uint8_t severity_mask);
  void setIncludeFilters(const QStringList &list);
  void setExcludeFilters(const QStringList &list);
  void setIncludeRegexpPattern(const QString& pattern);
  void setExcludeRegexpPattern(const QString& pattern);
  void setDebugColor(const QColor& debug_color);
  void setInfoColor(const QColor& info_color);
  void setWarnColor(const QColor& warn_color);
  void setErrorColor(const QColor& error_color);
  void setFatalColor(const QColor& fatal_color);
  bool isIncludeValid() const;
  bool isExcludeValid() const;


  virtual int rowCount(const QModelIndex &parent) const;
  virtual QVariant data(const QModelIndex &index, int role) const;

  void reset();

  void saveToFile(const QString& filename) const;

 Q_SIGNALS:
  void messagesAdded();

 public Q_SLOTS:
  void handleDatabaseCleared();
  void processNewMessages();
  void processOldMessages();
  void minTimeUpdated();
  void setDisplayTime(bool display);
  void setAbsoluteTime(bool absolute);
  void setColorizeLogs(bool colorize_logs);
  void setUseRegularExpressions(bool useRegexps);

 private:
  LogDatabase *db_;

  void saveBagFile(const QString& filename) const;
  void saveTextFile(const QString& filename) const;
  void scheduleIdleProcessing();
  
  bool acceptLogEntry(const LogEntry &item);
  bool testIncludeFilter(const LogEntry &item);
  
  std::set<std::string> names_;
  uint8_t severity_mask_;
  bool colorize_logs_;
  bool display_time_;
  bool display_absolute_time_;
  bool use_regular_expressions_;

  // For performance reasons, the proxy model presents single line
  // items, while the underlying log database stores multi-line
  // messages.  The LineMap struct is used to map our item indices to
  // the log & line that it represents.
  struct LineMap {
    size_t log_index;
    int line_index;

    LineMap() : log_index(0), line_index(0) {}
    LineMap(size_t log, int line) : log_index(log), line_index(line) {}
  };
  
  size_t latest_log_index_;
  std::deque<LineMap> msg_mapping_;

  size_t earliest_log_index_;
  std::deque<LineMap> early_mapping_;

  QRegExp include_regexp_;
  QRegExp exclude_regexp_;
  QStringList include_strings_;
  QStringList exclude_strings_;

  QColor debug_color_;
  QColor info_color_;
  QColor warn_color_;
  QColor error_color_;
  QColor fatal_color_;
};
}  // swri_console
#endif  // SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_
