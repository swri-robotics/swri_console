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

  void clear();

  void reset();

  void saveToFile(const QString& filename) const;

 Q_SIGNALS:
  void messagesAdded();
                      
 public Q_SLOTS:
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

  size_t earliest_index_;
  size_t latest_index_;
  
  std::set<std::string> names_;
  uint8_t severity_mask_;
  bool colorize_logs_;
  bool display_time_;
  bool display_absolute_time_;
  bool use_regular_expressions_;

  std::deque<size_t> early_mapping_;
  std::deque<size_t> msg_mapping_;

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
