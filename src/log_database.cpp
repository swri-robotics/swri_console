#include <swri_console/log_database.h>

namespace swri_console
{
LogDatabase::LogDatabase()
{
}

LogDatabase::~LogDatabase()
{
}

void LogDatabase::queueMessage(const rosgraph_msgs::Log &msg)
{
  nodes_[msg.name]++;

  LogEntry log;
  log.stamp = msg.header.stamp;
  log.level = msg.level;
  log.node = msg.name;
  log.file = msg.file;
  log.function = msg.function;
  log.line = msg.line;
  log.msg = msg.msg;
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
