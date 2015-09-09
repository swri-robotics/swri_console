#include <swri_console/log_database_proxy_model.h>
#include <swri_console/log_database.h>
#include <QTimer>
#include <stdio.h>
#include <ros/time.h>

namespace swri_console
{
LogDatabaseProxyModel::LogDatabaseProxyModel(LogDatabase *db)
  :
  db_(db)
{  
  QObject::connect(db_, SIGNAL(messagesAdded()),
                   this, SLOT(processNewMessages()));
}

LogDatabaseProxyModel::~LogDatabaseProxyModel()
{
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

int LogDatabaseProxyModel::rowCount(const QModelIndex &parent) const
{
  if (parent.isValid()) {
    return 0;
  }

  return msg_mapping_.size();
}

QVariant LogDatabaseProxyModel::data(
  const QModelIndex &index, int role) const
{
  if (index.parent().isValid() &&
      index.row() >= msg_mapping_.size()) {
    return QVariant();
  }
    
  const LogEntry &item = db_->log()[msg_mapping_[index.row()]];

  if (role == Qt::DisplayRole) {
    return QVariant(QString(item.msg.c_str()));
  }   
  
  return QVariant();
}

void LogDatabaseProxyModel::reset()
{
  beginResetModel();
  msg_mapping_.clear();
  early_mapping_.clear();
  earliest_index_ = db_->log().size();
  latest_index_ = earliest_index_;
  endResetModel();
  scheduleIdleProcessing();
}

void LogDatabaseProxyModel::processNewMessages()
{
  std::deque<size_t> new_items;
 
  // Process all messages from latest_index_ to the end of the
  // log.
  for (; 
       latest_index_ < db_->log().size();
       latest_index_++)
  {
    const LogEntry &item = db_->log()[latest_index_];    
    if (!acceptLogEntry(item)) {
      continue;
    }    
    new_items.push_back(latest_index_);
  }
  
  if (new_items.size()) {
    beginInsertRows(QModelIndex(),
                    msg_mapping_.size(),
                    msg_mapping_.size() + new_items.size() - 1);
    msg_mapping_.insert(msg_mapping_.end(),
                        new_items.begin(),
                        new_items.end());
    endInsertRows();

    Q_EMIT messagesAdded();
  }  
}

void LogDatabaseProxyModel::processOldMessages()
{
  //std::deque<size_t> new_items;
  
  for (size_t i = 0;
       earliest_index_ != 0 && i < 100;
       earliest_index_--, i++)
  {
    const LogEntry &item = db_->log()[earliest_index_-1];
    if (!acceptLogEntry(item)) {
      continue;
    }    
    early_mapping_.push_back(earliest_index_-1);
  }
 
  if ((earliest_index_ == 0 && early_mapping_.size()) ||
      (early_mapping_.size() > 200)) {
    beginInsertRows(QModelIndex(),
                    0,
                    early_mapping_.size() - 1);
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
  if (earliest_index_ > 0) {
    QTimer::singleShot(0, this, SLOT(processOldMessages()));
  }
}

bool LogDatabaseProxyModel::acceptLogEntry(const LogEntry &item)
{
  if (!(item.level & severity_mask_)) {
    return false;
  }
  
  if (names_.count(item.node) == 0) {
    return false;
  }

  return true;
}
}  // namespace swri_console
