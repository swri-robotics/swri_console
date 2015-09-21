#include <stdio.h>
#include <swri_console/node_list_model.h>
#include <vector>

namespace swri_console
{
NodeListModel::NodeListModel()
{
}

NodeListModel::~NodeListModel()
{
}

int NodeListModel::rowCount(const QModelIndex &parent) const
{
  return ordering_.size();
}

std::string NodeListModel::nodeName(const QModelIndex &index) const
{
  if (index.parent().isValid() ||
      index.row() > ordering_.size()) {
    return "";
  }

  return ordering_[index.row()];
}

QVariant NodeListModel::data(const QModelIndex &index, int role) const
{
  if (index.parent().isValid() ||
      index.row() > ordering_.size()) {
    return QVariant();
  } 

  std::string name = ordering_[index.row()];
  
  if (role == Qt::DisplayRole) {
    char buffer[1023];
    snprintf(buffer, sizeof(buffer), "%s (%lu)",
             name.c_str(),
             data_.find(name)->second);
    return QVariant(QString(buffer));
  }

  return QVariant();
}

void NodeListModel::update(const std::map<std::string, size_t> &updated_counts)  
{
  size_t i = 0;
  for (std::map<std::string, size_t>::const_iterator it = updated_counts.begin();
       it != updated_counts.end();
       ++it)
  {
    if (!data_.count(it->first)) {
      beginInsertRows(QModelIndex(), i, i);
      data_[it->first] = it->second;
      ordering_.insert(ordering_.begin() + i, it->first);
      endInsertRows();
    } else {
      data_[it->first] = it->second;
    }
    i++;
  }
  
  Q_EMIT dataChanged(index(0),
                     index(ordering_.size()-1));
}

void NodeListModel::clear()
{
  unsigned long size = data_.size();
  beginRemoveRows(index(0), 0, size-1);
  data_.clear();
  ordering_.clear();
  endRemoveRows();
}

void NodeListModel::clearLogs()
{
  std::map<std::string, size_t>::iterator iter;
  for (iter = data_.begin(); iter != data_.end(); ++iter)
  {
    (*iter).second = 0;
  }

  Q_EMIT dataChanged(index(0), index(data_.size()-1));
}
}  // namespace swri_console
