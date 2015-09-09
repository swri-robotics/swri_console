#include <swri_console/console_window.h>
#include <stdint.h>
#include <set>
#include <swri_console/log_database.h>
#include <swri_console/log_database_proxy_model.h>
#include <rosgraph_msgs/Log.h>

using namespace Qt;

namespace swri_console {

ConsoleWindow::ConsoleWindow(LogDatabase *db)
  :
  QMainWindow(),
  db_(db),
  db_proxy_(new LogDatabaseProxyModel(db))
{
  ui.setupUi(this); 

  QObject::connect(ui.action_NewWindow, SIGNAL(triggered(bool)),
                   this, SIGNAL(createNewWindow()));
  
  ui.nodeList->setModel(db_->nodeListModel());
  ui.messageList->setModel(db_proxy_);
  
  QObject::connect(
    ui.nodeList->selectionModel(),
    SIGNAL(selectionChanged(const QItemSelection &,
                                const QItemSelection &)),
    this,
    SLOT(nodeSelectionChanged()));

  QObject::connect(
    ui.checkDebug, SIGNAL(toggled(bool)),
    this, SLOT(setSeverityFilter()));
  QObject::connect(
    ui.checkInfo, SIGNAL(toggled(bool)),
    this, SLOT(setSeverityFilter()));
  QObject::connect(
    ui.checkWarn, SIGNAL(toggled(bool)),
    this, SLOT(setSeverityFilter()));
  QObject::connect(
    ui.checkError, SIGNAL(toggled(bool)),
    this, SLOT(setSeverityFilter()));
  QObject::connect(
    ui.checkFatal, SIGNAL(toggled(bool)),
    this, SLOT(setSeverityFilter()));
  QObject::connect(
    db_proxy_, SIGNAL(messagesAdded()),
    this, SLOT(messagesAdded()));    
  
  setSeverityFilter();
}

ConsoleWindow::~ConsoleWindow()
{
  delete db_proxy_;
}

void ConsoleWindow::connected(bool connected)
{
  if (connected) {
    statusBar()->showMessage("Connected to ROS Master");
  } else {
    statusBar()->showMessage("Disconnected from ROS Master");
  }
}

void ConsoleWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void ConsoleWindow::nodeSelectionChanged()
{
  QModelIndexList selection = ui.nodeList->selectionModel()->selectedIndexes();
  std::set<std::string> nodes;

  for (size_t i = 0; i < selection.size(); i++) {
    std::string name = db_->nodeListModel()->nodeName(selection[i]);
    nodes.insert(name);
  }

  db_proxy_->setNodeFilter(nodes);
}

void ConsoleWindow::setSeverityFilter()
{
  uint8_t mask = 0;

  if (ui.checkDebug->isChecked()) {
    mask |= rosgraph_msgs::Log::DEBUG;
  }
  if (ui.checkInfo->isChecked()) {
    mask |= rosgraph_msgs::Log::INFO;
  }
  if (ui.checkWarn->isChecked()) {
    mask |= rosgraph_msgs::Log::WARN;
  }
  if (ui.checkError->isChecked()) {
    mask |= rosgraph_msgs::Log::ERROR;
  }
  if (ui.checkFatal->isChecked()) {
    mask |= rosgraph_msgs::Log::FATAL;
  }

  db_proxy_->setSeverityFilter(mask);
}

void ConsoleWindow::messagesAdded()
{
  if (ui.checkFollowNewest->isChecked()) {
    ui.messageList->scrollToBottom();
  }
}
}  // namespace swri_console

