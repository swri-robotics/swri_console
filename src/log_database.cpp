#include <swri_console/log_database.h>

namespace swri_console
{
LogDatabase::LogDatabase()
  :
  min_time_(ros::TIME_MAX)
{
}

LogDatabase::~LogDatabase()
{
}

void LogDatabase::clear()
{
  std::map<std::string, size_t>::iterator iter;
  // Set the log count for the node list to 0.
  for (iter = nodes_.begin(); iter != nodes_.end(); iter++)
  {
    (*iter).second = 0;
  }
  // Tell the node list model to also clear out its internal storage.
  node_list_model_.clearLogs();
  // Finally, remove all of the logs we've stored.
  log_.clear();
}

void LogDatabase::queueMessage(const rosgraph_msgs::LogConstPtr msg)
{
  if (msg->header.stamp < min_time_) {
    min_time_ = msg->header.stamp;
    Q_EMIT minTimeUpdated();
  }
  
  nodes_[msg->name]++;

  LogEntry log;
  log.stamp = msg->header.stamp;
  log.level = msg->level;
  log.node = msg->name;
  log.file = msg->file;
  log.function = msg->function;
  log.line = msg->line;
  log.msg = QString(msg->msg.c_str());
  new_msgs_.push_back(log);    
}

void LogDatabase::processQueue()
{
  if (new_msgs_.empty()) {
    return;
  }
  
  node_list_model_.update(nodes_);
  log_.insert(log_.end(),
              new_msgs_.begin(),
              new_msgs_.end());
  new_msgs_.clear();

  Q_EMIT messagesAdded();              
}
}  // namespace swri_console
