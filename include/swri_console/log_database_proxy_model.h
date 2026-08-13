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

#ifndef SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_
#define SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_

#include <QAbstractListModel>
#include <QColor>
#include <QRegularExpression>
#include <QStringList>

#include <rclcpp/time.hpp>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <deque>

namespace swri_console
{

class LogDatabase;
struct LogEntry;

class LogDatabaseProxyModel : public QAbstractListModel
{
  Q_OBJECT
 
 public:
  enum {
    ExtendedLogRole = Qt::UserRole + 0
  };

  explicit LogDatabaseProxyModel(LogDatabase *db);
  ~LogDatabaseProxyModel() override = default;

  void setNodeFilter(const std::set<std::string> &names);
  void setSeverityFilter(uint8_t severity_mask);
  void setIncludeFilters(const QStringList &list);
  void setExcludeFilters(const QStringList &list);
  void setIncludeRegexpPattern(const QString& pattern);
  void setExcludeRegexpPattern(const QString& pattern);
  void setExcludePreviewFilter(const QString& term);
  void setOutputFormat(const QString& format);
  const QString& outputFormat() const { return output_format_; }
  void setHighlightFilters(const QStringList &list);
  void setHighlightRegexpPattern(const QString& pattern);
  void setHighlightColor(const QColor& highlight_color);
  bool isHighlightValid() const;
  void setDebugColor(const QColor& debug_color);
  void setInfoColor(const QColor& info_color);
  void setWarnColor(const QColor& warn_color);
  void setErrorColor(const QColor& error_color);
  void setFatalColor(const QColor& fatal_color);
  void setNodeColor(const std::string& node, const QColor& color);
  void clearNodeColor(const std::string& node);
  const std::map<std::string, QColor>& nodeColors() const { return node_colors_; }
  bool isIncludeValid() const;
  bool isExcludeValid() const;
  int getItemIndex(const QString& searchText, int index, int increment);
  void clearSearchFailure();

  int rowCount(const QModelIndex &parent) const override;
  QVariant data(const QModelIndex &index, int role) const override;

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
  void setHumanReadableTime(bool human_readable_time);
  void setDisplayLogger(bool logger_name);
  void setDisplayFunction(bool function_name);
  void setColorizeLogs(bool colorize_logs);
  void setUseRegularExpressions(bool useRegexps);

 private:
  void saveBagFile(const QString& filename) const;
  void saveTextFile(const QString& filename) const;
  void scheduleIdleProcessing();
  
  bool acceptLogEntry(const LogEntry &item);
  bool testIncludeFilter(const LogEntry &item);
  bool matchesExcludePreview(const LogEntry &item) const;
  bool matchesHighlight(const LogEntry &item) const;
  void repaintBackgrounds();
  void formatTimestamp(const rclcpp::Time &stamp, char *buf, size_t size) const;
  void parseOutputFormat();
  int entryLineCount(const LogEntry &item) const;
  QString formatCustomLine(const LogEntry &item, int line_index) const;
  QString substituteTokens(const QString &line_template, const LogEntry &item, const QString &message) const;
  
  std::set<std::string> names_;
  uint8_t severity_mask_;
  bool colorize_logs_;
  bool display_time_;
  bool display_absolute_time_;
  bool human_readable_time_;
  bool display_logger_;
  bool display_function_;
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

  QRegularExpression include_regexp_;
  QRegularExpression exclude_regexp_;
  QStringList include_strings_;
  QStringList exclude_strings_;

  // The exclude term currently being typed (not yet committed to
  // exclude_strings_/exclude_regexp_).  Matching rows are highlighted
  // rather than filtered out until the term is committed.
  QString exclude_preview_term_;
  QRegularExpression exclude_preview_regexp_;

  // Highlight filter: unlike include/exclude, matching rows are never
  // hidden -- they're just tinted highlight_color_ via Qt::BackgroundRole,
  // same mechanism as exclude_preview_term_ above but persistent rather
  // than typing-only.
  QRegularExpression highlight_regexp_;
  QStringList highlight_strings_;
  QColor highlight_color_;

  // When non-empty, overrides the display_time_/display_logger_/display_function_
  // driven layout entirely.  Supports the same token names as ROS 2's
  // RCUTILS_CONSOLE_OUTPUT_FORMAT: {severity} {name} {function_name} {file_name}
  // {line_number} {time} {message}.  May span multiple lines: the one line
  // containing {message} is repeated once per physical line of the log
  // message; any lines before/after it are rendered once per log entry as
  // a header/footer.  Derived from output_format_ by parseOutputFormat()
  // whenever it changes.
  QString output_format_;
  QStringList format_pre_lines_;
  QString format_message_line_;
  QStringList format_post_lines_;

  QColor debug_color_;
  QColor info_color_;
  QColor warn_color_;
  QColor error_color_;
  QColor fatal_color_;

  // Per-node background tint, set via the node list's right-click "Select
  // Color..." menu.  Absent entries mean "no override for this node".
  std::map<std::string, QColor> node_colors_;

  LogDatabase *db_;

  QString failedSearchText_;  // stores last failed search text, used to minimize looping through full data set, VCM 26 April 2017
  int failedSearchIndex_;  // stores last index of failed search text, VCM 26 April 2017

};
}  // swri_console
#endif  // SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_
