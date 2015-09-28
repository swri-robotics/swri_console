#include <stdint.h>
#include <stdio.h>
#include <set>

#include <rosgraph_msgs/Log.h>

#include <swri_console/console_window.h>
#include <swri_console/log_database.h>
#include <swri_console/log_database_proxy_model.h>

#include <rosgraph_msgs/Log.h>

#include <QColorDialog>
#include <QRegExp>
#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QFileDialog>
#include <QDir>
#include <QScrollBar>
#include <QMenu>

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

  QObject::connect(ui.action_Copy, SIGNAL(triggered()),
                   this, SLOT(copyLogs()));

  QObject::connect(ui.action_CopyExtended, SIGNAL(triggered()),
                   this, SLOT(copyExtendedLogs()));
  
  QObject::connect(ui.action_SelectAll, SIGNAL(triggered()),
                   this, SLOT(selectAllLogs()));

  QObject::connect(ui.action_ReadBagFile, SIGNAL(triggered(bool)),
                   this, SIGNAL(readBagFile()));

  QObject::connect(ui.action_SaveLogs, SIGNAL(triggered(bool)),
                   this, SLOT(saveLogs()));

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

  QObject::connect(ui.action_ColorizeLogs, SIGNAL(triggered(bool)),
                   db_proxy_, SLOT(setColorizeLogs(bool)));

  QObject::connect(ui.debugColorWidget, SIGNAL(clicked(bool)),
                   this, SLOT(setDebugColor()));
  QObject::connect(ui.infoColorWidget, SIGNAL(clicked(bool)),
                   this, SLOT(setInfoColor()));
  QObject::connect(ui.warnColorWidget, SIGNAL(clicked(bool)),
                   this, SLOT(setWarnColor()));
  QObject::connect(ui.errorColorWidget, SIGNAL(clicked(bool)),
                   this, SLOT(setErrorColor()));
  QObject::connect(ui.fatalColorWidget, SIGNAL(clicked(bool)),
                   this, SLOT(setFatalColor()));

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

  // Right-click menu for the message list
  QObject::connect(ui.messageList, SIGNAL(customContextMenuRequested(const QPoint&)),
                    this, SLOT(showLogContextMenu(const QPoint&)));
  QObject::connect(ui.clearLogsButton, SIGNAL(clicked()),
                    this, SLOT(clearLogs()));
  QObject::connect(ui.clearNodeListButton, SIGNAL(clicked()),
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

  // TODO pjreed Read these from the settings after user settings are implemented.
  updateButtonColor(ui.debugColorWidget, Qt::gray);
  updateButtonColor(ui.infoColorWidget, Qt::black);
  updateButtonColor(ui.warnColorWidget, QColor(255, 127, 0));
  updateButtonColor(ui.errorColorWidget, Qt::red);
  updateButtonColor(ui.fatalColorWidget, Qt::magenta);
}

ConsoleWindow::~ConsoleWindow()
{
  delete db_proxy_;
}

void ConsoleWindow::clearLogs()
{
  db_proxy_->clear();
}

void ConsoleWindow::saveLogs()
{
  QString defaultname = QDateTime::currentDateTime().toString(Qt::ISODate) + ".bag";
  QString filename = QFileDialog::getSaveFileName(this,
                                                  "Save Logs",
                                                  QDir::homePath() + QDir::separator() + defaultname,
                                                  tr("Bag Files (*.bag);;Text Files (*.txt)"));
  if (filename != NULL && !filename.isEmpty()) {
    db_proxy_->saveToFile(filename);
  }
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


void ConsoleWindow::showLogContextMenu(const QPoint& point)
{
  QMenu contextMenu(tr("Context menu"), ui.messageList);

  QAction selectAll(tr("Select All"), ui.messageList);
  connect(&selectAll, SIGNAL(triggered()), this, SLOT(selectAllLogs()));
  QAction copy(tr("Copy"), ui.messageList);
  connect(&copy, SIGNAL(triggered()), this, SLOT(copyLogs()));
  QAction copy_extended(tr("Copy Extended"), ui.messageList);
  connect(&copy_extended, SIGNAL(triggered()), this, SLOT(copyExtendedLogs()));
  
  contextMenu.addAction(&selectAll);
  contextMenu.addAction(&copy);
  contextMenu.addAction(&copy_extended);

  contextMenu.exec(ui.messageList->mapToGlobal(point));
}

void ConsoleWindow::userScrolled(int value)
{
  if (value != ui.messageList->verticalScrollBar()->maximum()) {
    ui.checkFollowNewest->setChecked(false);
  } else {
    ui.checkFollowNewest->setChecked(true);
  }
}


void ConsoleWindow::selectAllLogs()
{
  ui.messageList->selectAll();
}

void ConsoleWindow::copyLogs()
{
  QStringList buffer;
  foreach(const QModelIndex &index, ui.messageList->selectionModel()->selectedIndexes())
  {
    buffer << db_proxy_->data(index, Qt::DisplayRole).toString();
  }
  QApplication::clipboard()->setText(buffer.join(tr("\n")));
}

void ConsoleWindow::copyExtendedLogs()
{
  QStringList buffer;
  foreach(const QModelIndex &index, ui.messageList->selectionModel()->selectedIndexes())
  {
    buffer << db_proxy_->data(index, LogDatabaseProxyModel::ExtendedLogRole).toString();
  }
  QApplication::clipboard()->setText(buffer.join(tr("\n\n")));
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

void ConsoleWindow::setDebugColor()
{
  chooseButtonColor(ui.debugColorWidget);
}

void ConsoleWindow::setInfoColor()
{
  chooseButtonColor(ui.infoColorWidget);
}

void ConsoleWindow::setWarnColor()
{
  chooseButtonColor(ui.warnColorWidget);
}

void ConsoleWindow::setErrorColor()
{
  chooseButtonColor(ui.errorColorWidget);
}

void ConsoleWindow::setFatalColor()
{
  chooseButtonColor(ui.fatalColorWidget);
}

void ConsoleWindow::chooseButtonColor(QPushButton* widget)
{
  QString ss = widget->styleSheet();
  QRegExp re("background: (#\\w*);");
  QColor old_color;
  if (re.indexIn(ss) >= 0) {
    old_color = QColor(re.cap(1));
  }
  QColor color = QColorDialog::getColor(old_color, this);
  if (color.isValid()) {
    updateButtonColor(widget, color);
  }
}

void ConsoleWindow::updateButtonColor(QPushButton* widget, const QColor& color)
{
  QString s("background: #"
            + QString(color.red() < 16? "0" : "") + QString::number(color.red(),16)
            + QString(color.green() < 16? "0" : "") + QString::number(color.green(),16)
            + QString(color.blue() < 16? "0" : "") + QString::number(color.blue(),16) + ";");
  widget->setStyleSheet(s);
  widget->update();

  if (widget == ui.debugColorWidget) {
    db_proxy_->setDebugColor(color);
  }
  else if (widget == ui.infoColorWidget) {
    db_proxy_->setInfoColor(color);
  }
  else if (widget == ui.warnColorWidget) {
    db_proxy_->setWarnColor(color);
  }
  else if (widget == ui.errorColorWidget) {
    db_proxy_->setErrorColor(color);
  }
  else if (widget == ui.fatalColorWidget) {
    db_proxy_->setFatalColor(color);
  }
  else {
    qWarning("Unexpected widget passed to ConsoleWindow::updateButtonColor.");
  }
}
}  // namespace swri_console

