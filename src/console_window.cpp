#include <swri_console/console_window.h>
#include <stdint.h>
#include <stdio.h>
#include <set>
#include <swri_console/log_database.h>
#include <swri_console/log_database_proxy_model.h>
#include <rosgraph_msgs/Log.h>
#include <QScrollBar>

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

  QObject::connect(ui.action_AbsoluteTimestamps, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setAbsoluteTime(bool)));

  QObject::connect(ui.action_ShowTimestamps, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setDisplayTime(bool)));

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setUseRegularExpressions(bool)));

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   this, SLOT(updateIncludeLabel()));

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   this, SLOT(updateExcludeLabel()));

  QObject::connect(ui.action_SelectFont, SIGNAL(triggered(bool)),
                   this, SIGNAL(selectFont()));
  
  ui.nodeList->setModel(db_->nodeListModel());
  ui.messageList->setModel(db_proxy_);
  ui.messageList->setUniformItemSizes(true);
  
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

  QObject::connect( ui.clearLogsButton, SIGNAL(clicked()),
                    this, SLOT(clearLogs()));
  QObject::connect( ui.clearNodeListButton, SIGNAL(clicked()),
                    this, SLOT(clearNodes()));

  QObject::connect(
    ui.messageList->verticalScrollBar(), SIGNAL(valueChanged(int)),
    this, SLOT(userScrolled(int)));

  QObject::connect(
    ui.includeText, SIGNAL(textChanged(const QString &)),
    this, SLOT(includeFilterUpdated(const QString &)));

  QObject::connect(
    ui.excludeText, SIGNAL(textChanged(const QString &)),
    this, SLOT(excludeFilterUpdated(const QString &)));

  QList<int> sizes;
  sizes.append(100);
  sizes.append(1000);
  ui.splitter->setSizes(sizes);

  db_proxy_->setDisplayTime(true);
  setSeverityFilter();
}

ConsoleWindow::~ConsoleWindow()
{
  delete db_proxy_;
}

void ConsoleWindow::clearLogs()
{
  db_proxy_->clear();
}


void ConsoleWindow::clearNodes()
{
  db_proxy_->clear();
  db_->nodeListModel()->clear();
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
  QStringList node_names;

  for (size_t i = 0; i < selection.size(); i++) {
    std::string name = db_->nodeListModel()->nodeName(selection[i]);
    nodes.insert(name);
    node_names.append(name.c_str());
  }

  db_proxy_->setNodeFilter(nodes);

  for (size_t i = 0; i < node_names.size(); i++) {
    node_names[i] = node_names[i].split("/", QString::SkipEmptyParts).last();
  }
    
  setWindowTitle(QString("SWRI Console (") + node_names.join(", ") + ")");
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

void ConsoleWindow::userScrolled(int value)
{
  if (value != ui.messageList->verticalScrollBar()->maximum()) {
    ui.checkFollowNewest->setChecked(false);
  } else {
    ui.checkFollowNewest->setChecked(true);
  }
}

void ConsoleWindow::includeFilterUpdated(const QString &text)
{
  QStringList items = text.split(";", QString::SkipEmptyParts);
  QStringList filtered;
  
  for (int i = 0; i < items.size(); i++) {
    QString x = items[i].trimmed();
    if (!x.isEmpty()) {
      filtered.append(x);
    }
  }

  db_proxy_->setIncludeFilters(filtered);
  db_proxy_->setIncludeRegexpPattern(text);
  updateIncludeLabel();
}

void ConsoleWindow::excludeFilterUpdated(const QString &text)
{
  QStringList items = text.split(";", QString::SkipEmptyParts);
  QStringList filtered;
  
  for (int i = 0; i < items.size(); i++) {
    QString x = items[i].trimmed();
    if (!x.isEmpty()) {
      filtered.append(x);
    }
  }

  db_proxy_->setExcludeFilters(filtered);
  db_proxy_->setExcludeRegexpPattern(text);
  updateExcludeLabel();
}


void ConsoleWindow::updateIncludeLabel()
{
  if (db_proxy_->isIncludeValid()) {
    ui.includeLabel->setText("Include");
  } else {
    ui.includeLabel->setText("<font color='red'>Include</font>");
  }
}

void ConsoleWindow::updateExcludeLabel()
{
  if (db_proxy_->isExcludeValid()) {
    ui.excludeLabel->setText("Exclude");
  } else {
    ui.excludeLabel->setText("<font color='red'>Exclude</font>");
  }
}

void ConsoleWindow::setFont(const QFont &font)
{
  ui.messageList->setFont(font);
  ui.nodeList->setFont(font);
}
}  // namespace swri_console

