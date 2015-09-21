#ifndef SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_
#define SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_

#include <QAbstractListModel>
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
  LogDatabaseProxyModel(LogDatabase *db);
  ~LogDatabaseProxyModel();

  void setNodeFilter(const std::set<std::string> &names);
  void setSeverityFilter(uint8_t severity_mask);
  void setIncludeFilters(const QStringList &list);
  void setExcludeFilters(const QStringList &list);
  void setIncludeRegexpPattern(const QString& pattern);
  void setExcludeRegexpPattern(const QString& pattern);
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
  bool display_time_;
  bool display_absolute_time_;
  bool use_regular_expressions_;

  std::deque<size_t> early_mapping_;
  std::deque<size_t> msg_mapping_;

  QRegExp include_regexp_;
  QRegExp exclude_regexp_;
  QStringList include_strings_;
  QStringList exclude_strings_;
};
}  // swri_console
#endif  // SWRI_CONSOLE_LOG_DATABASE_PROXY_MODEL_H_
