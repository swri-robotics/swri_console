#ifndef SWRI_CONSOLE_LOG_DATABASE_H_
#define SWRI_CONSOLE_LOG_DATABASE_H_

#include <QObject>
#include <QAbstractListModel>
#include <rosgraph_msgs/Log.h>
#include <swri_console/node_list_model.h>
#include <deque>
#include <ros/time.h>

namespace swri_console
{
struct LogEntry
{
  ros::Time stamp;
  uint8_t level;  
  std::string node;  
  std::string file;
  std::string function;
  uint32_t line;
  std::string msg;
};

class LogDatabase : public QObject
{
  Q_OBJECT
  
public:
  LogDatabase();
  ~LogDatabase();
  
  NodeListModel *nodeListModel() { return &node_list_model_; }
  const std::deque<LogEntry>& log() { return log_; }
  const ros::Time& minTime() const { return min_time_; }

  void queueMessage(const rosgraph_msgs::Log &msg);
  void processQueue();

 Q_SIGNALS:
  void messagesAdded();
  void minTimeUpdated();
  
private:  
  std::map<std::string, size_t> nodes_;
  std::deque<LogEntry> log_;
  std::deque<LogEntry> new_msgs_;

  ros::Time min_time_;
  
  NodeListModel node_list_model_;
};  // class LogDatabase
}  // namespace swri_console 
#endif  // SWRI_CONSOLE_LOG_DATABASE_H_
