#ifndef SWRI_CONSOLE_NODE_LIST_MODEL_H_
#define SWRI_CONSOLE_NODE_LIST_MODEL_H_

#include <string>
#include <vector>
#include <map>
#include <QAbstractListModel>

namespace swri_console
{
class NodeListModel : public QAbstractListModel
{
  Q_OBJECT
  
 public:
  NodeListModel();
  ~NodeListModel();

  std::string nodeName(const QModelIndex &index) const;
  
  virtual int rowCount(const QModelIndex &parent) const;
  virtual QVariant data(const QModelIndex &index, int role) const;
  
  void clear();
  void update(const std::map<std::string, size_t> &updated_counts);

 private:
  std::map<std::string, size_t> data_;
  std::vector<std::string> ordering_;
};
}  // namespace swri_console
#endif  // SWRI_CONSOLE_NODE_LIST_MODEL_H_
