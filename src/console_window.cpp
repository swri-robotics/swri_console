// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <cstdint>
#include <cstdio>
#include <set>

// #include <rosgraph_msgs/Log.h>
// #include <ros/master.h>  // required for getURI, VCM 12 April 2017
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <swri_console/console_window.h>
#include <swri_console/log_database.h>
#include <swri_console/log_database_proxy_model.h>
#include <swri_console/node_list_model.h>
#include <swri_console/settings_keys.h>
#include <swri_console/defines.h>

#include <QColorDialog>
#include <QRegExp>
#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QFileDialog>
#include <QDir>
#include <QScrollBar>
#include <QMenu>
#include <QSettings>

// QString::SkipEmptyParts was deprecated in favor of Qt::SkipEmptyParts in
// Qt 5.14.0
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
  #define SPLIT_FLAG (Qt::SkipEmptyParts)
#else
  #define SPLIT_FLAG (QString::SkipEmptyParts)
#endif

using namespace Qt;

namespace swri_console {

ConsoleWindow::ConsoleWindow(LogDatabase *db)
  : QMainWindow()
  , searchFunction_(SEARCH)
  , ui()
  , db_(db)
  , db_proxy_(new LogDatabaseProxyModel(db))
  , node_list_model_(new NodeListModel(db))
  , node_click_handler_(new NodeClickHandler())
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

  QObject::connect(ui.action_ReadLogFile, SIGNAL(triggered(bool)),
                   this, SIGNAL(readLogFile()));

  QObject::connect(ui.action_ReadLogDirectory, SIGNAL(triggered(bool)),
                   this, SIGNAL(readLogDirectory()));

  QObject::connect(ui.action_SaveLogs, SIGNAL(triggered(bool)),
                   this, SLOT(saveLogs()));

  QObject::connect(ui.action_AbsoluteTimestamps, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setAbsoluteTime(bool)));

  QObject::connect(ui.action_ShowTimestamps, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setDisplayTime(bool)));

  QObject::connect(ui.action_ShowLoggerName, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setDisplayLogger(bool)));

  QObject::connect(ui.action_ShowFunctionName, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setDisplayFunction(bool)));

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setUseRegularExpressions(bool)));

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   this, SLOT(updateIncludeLabel()));

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   this, SLOT(updateExcludeLabel()));

  QObject::connect(ui.action_SelectFont, SIGNAL(triggered(bool)),
                   this, SIGNAL(selectFont()));

  QObject::connect(ui.action_ColorizeLogs, SIGNAL(toggled(bool)),
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

  ui.nodeList->setModel(node_list_model_);  
  ui.messageList->setModel(db_proxy_);
  ui.messageList->setUniformItemSizes(true);

  QObject::connect(
    ui.nodeList->selectionModel(),
    SIGNAL(selectionChanged(const QItemSelection &,
                                const QItemSelection &)),
    this,
    SLOT(nodeSelectionChanged()));

  ui.nodeList->installEventFilter(node_click_handler_);

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
  QObject::connect(ui.checkFollowNewest, SIGNAL(toggled(bool)),
                   this, SLOT(setFollowNewest(bool)));

  // Right-click menu for the message list
  QObject::connect(ui.messageList, SIGNAL(customContextMenuRequested(const QPoint&)),
                    this, SLOT(showLogContextMenu(const QPoint&)));

  QObject::connect(ui.clearAllButton, SIGNAL(clicked()),
                    this, SLOT(clearAll()));
  QObject::connect(ui.clearMessagesButton, SIGNAL(clicked()),
                    this, SLOT(clearMessages()));

  QObject::connect(
    ui.messageList->verticalScrollBar(), SIGNAL(valueChanged(int)),
    this, SLOT(userScrolled(int)));

  QObject::connect(
    ui.includeText, SIGNAL(textChanged(const QString &)),
    this, SLOT(includeFilterUpdated(const QString &)));

  QObject::connect(
    ui.excludeText, SIGNAL(textChanged(const QString &)),
    this, SLOT(excludeFilterUpdated(const QString &)));

  // Connect 'Search' text modification to searchIndex, VCM 13 April 2017
  QObject::connect(
    ui.searchText, SIGNAL(textChanged(const QString &)),
    this, SLOT(searchIndex()));
  // Connect pushPrev to prevIndex()
  QObject::connect(ui.pushPrev, SIGNAL(clicked()),
    this, SLOT(prevIndex()));
  // Connect pushNext to nextIndex()
  QObject::connect(ui.pushNext, SIGNAL(clicked()),
    this, SLOT(nextIndex()));


  QList<int> sizes;
  sizes.append(100);
  sizes.append(1000);
  ui.splitter->setSizes(sizes);

  loadSettings();
}

ConsoleWindow::~ConsoleWindow()
{
  delete db_proxy_;
}

void ConsoleWindow::clearAll()
{
  db_->clear();
  node_list_model_->clear();
  db_proxy_->clearSearchFailure();  // resets failed search variables, VCM 27 April 2017
}

void ConsoleWindow::clearMessages()
{
  db_->clear();
  db_proxy_->clearSearchFailure();  // resets failed search variables, VCM 27 April 2017
}

void ConsoleWindow::saveLogs()
{
  QString defaultname = QDateTime::currentDateTime().toString(Qt::ISODate) + ".bag";
  QString filename = QFileDialog::getSaveFileName(this,
                                                  "Save Logs",
                                                  QDir::homePath() + QDir::separator() + defaultname,
                                                  tr("Bag Files (*.bag);;Text Files (*.txt)"));
  if (filename != nullptr && !filename.isEmpty()) {
    db_proxy_->saveToFile(filename);
  }
}

void ConsoleWindow::connected(bool connected)
{
  if (connected) {
    statusBar()->showMessage("Listening for logs.");
  } else {
    statusBar()->showMessage("ROS has shut down.");
  }
}

void ConsoleWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void ConsoleWindow::nodeSelectionChanged()
{
  db_proxy_->clearSearchFailure();  // clear search failure criteria, VCM 26 April 2017
  QModelIndexList selection = ui.nodeList->selectionModel()->selectedIndexes();
  std::set<std::string> nodes;
  QStringList node_names;

  for (const auto & i : selection) {
    std::string name = node_list_model_->nodeName(i);
    nodes.insert(name);
    node_names.append(name.c_str());
  }

  db_proxy_->setNodeFilter(nodes);

  for (int i = 0; i < node_names.size(); i++) {
    node_names[i] = node_names[i].split("/", SPLIT_FLAG).last();
  }
    
  setWindowTitle(QString("SWRI Console (") + node_names.join(", ") + ")");
}

void ConsoleWindow::setSeverityFilter()
{
  uint8_t mask = 0;

  if (ui.checkDebug->isChecked()) {
    mask |= LogLevelMask::DEBUG;
  }
  if (ui.checkInfo->isChecked()) {
    mask |= LogLevelMask::INFO;
  }
  if (ui.checkWarn->isChecked()) {
    mask |= LogLevelMask::WARN;
  }
  if (ui.checkError->isChecked()) {
    mask |= LogLevelMask::ERROR;
  }
  if (ui.checkFatal->isChecked()) {
    mask |= LogLevelMask::FATAL;
  }

  QSettings settings;
  settings.setValue(SettingsKeys::SHOW_DEBUG, ui.checkDebug->isChecked());
  settings.setValue(SettingsKeys::SHOW_INFO, ui.checkInfo->isChecked());
  settings.setValue(SettingsKeys::SHOW_WARN, ui.checkWarn->isChecked());
  settings.setValue(SettingsKeys::SHOW_ERROR, ui.checkError->isChecked());
  settings.setValue(SettingsKeys::SHOW_FATAL, ui.checkFatal->isChecked());

  db_proxy_->setSeverityFilter(mask);
  db_proxy_->clearSearchFailure();  // resets search failure variables, VCM 27 April 2017
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

  QAction select_all(tr("Select All"), ui.messageList);
  connect(&select_all, SIGNAL(triggered()), this, SLOT(selectAllLogs()));

  QAction copy(tr("Copy"), ui.messageList);
  connect(&copy, SIGNAL(triggered()), this, SLOT(copyLogs()));

  QAction copy_extended(tr("Copy Extended"), ui.messageList);
  connect(&copy_extended, SIGNAL(triggered()), this, SLOT(copyExtendedLogs()));

  QAction alternate_row_colors(tr("Alternate Row Colors"), ui.messageList);
  alternate_row_colors.setCheckable(true);
  alternate_row_colors.setChecked(ui.messageList->alternatingRowColors());
  connect(&alternate_row_colors, SIGNAL(toggled(bool)),
          this, SLOT(toggleAlternateRowColors(bool)));
            
  contextMenu.addAction(&select_all);
  contextMenu.addAction(&copy);
  contextMenu.addAction(&copy_extended);
  contextMenu.addAction(&alternate_row_colors);

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
  if (ui.nodeList->hasFocus()) {
    ui.nodeList->selectAll();
  } else {
    ui.messageList->selectAll();
  }
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

void ConsoleWindow::setFollowNewest(bool follow)
{
  QSettings settings;
  settings.setValue(SettingsKeys::FOLLOW_NEWEST, follow);
}

void ConsoleWindow::includeFilterUpdated(const QString &text)
{
  QStringList items = text.split(";", SPLIT_FLAG);
  QStringList filtered;
  
  for (int i = 0; i < items.size(); i++) {
    QString x = items[i].trimmed();
    if (!x.isEmpty()) {
      filtered.append(x);
    }
  }

  db_proxy_->setIncludeFilters(filtered);
  db_proxy_->setIncludeRegexpPattern(text);
  db_proxy_->clearSearchFailure();  // resets failed search variables, VCM 27 April 2017
  updateIncludeLabel();
}

void ConsoleWindow::excludeFilterUpdated(const QString &text)
{
  QStringList items = text.split(";", SPLIT_FLAG);
  QStringList filtered;
  
  for (int i = 0; i < items.size(); i++) {
    QString x = items[i].trimmed();
    if (!x.isEmpty()) {
      filtered.append(x);
    }
  }

  db_proxy_->setExcludeFilters(filtered);
  db_proxy_->setExcludeRegexpPattern(text);
  db_proxy_->clearSearchFailure();  // resets failed search variables, VCM 27 April 2017
  updateExcludeLabel();
}

// Slot called when 'Search' text modified, 13 April 2017 VCM
void ConsoleWindow::searchIndex()
{
  updateCurrentIndex(SEARCH);
}
// Slot called when 'Previous' button pushed, 13 April 2017 VCM
void ConsoleWindow::prevIndex()
{
  updateCurrentIndex(PREV);
}
// Slot called when 'Next' button pushed,  13 April 2017 VCM
void ConsoleWindow::nextIndex()
{
  updateCurrentIndex(NEXT);
}

// Search Function sF Definitions:
//   1)search - user modified 'Search' text
//   2)next   - user pressed 'Next' button
//   3)prev   - user pressed 'Previous' button
// Locates and selects the next item based on search criteria, VCM 26 April 2017
void ConsoleWindow::updateCurrentIndex(function sF)
{
  int rowSearchStart = ui.messageList->currentIndex().row();  // retrieve current index
  int increment = 1;  // used for search/next/prev; prev(ious) increment will change to -1
  QString searchText = ui.searchText->text();  // actual text to search for
  searchText = searchText.toUpper().trimmed();  // remove lowercase and lead/trailing spaces.
  // next button pushed
  if(sF == NEXT){
    rowSearchStart++;  // start search row after current.
  }
  // Previous button pushed
  else if(sF== PREV){
    rowSearchStart--;  // start search row before current
    increment=-1;  // -1 to move search up instead of down
  }
  // search text modified
  else if(sF==SEARCH )
  {
    if (rowSearchStart==-1){
      rowSearchStart =0;  // for search, no selection (-1) index change to 0
    }
  }
  else
  {
    // should not end up here
    printf("Invalid string passed to ConsoleWindow::nextIndex");
    return;
  }
  // calls getItemIndex in log_database_proxy_m, returns new index
  int newRowIndex = db_proxy_->getItemIndex(searchText,rowSearchStart, increment);
  ui.messageList->clearSelection();  // clear current selection
  if(newRowIndex == -1)  // indicates no match.
  {
    return;
  }

  QModelIndex index = ui.messageList->model()->index(newRowIndex,0);  // defines desired index
  ui.messageList->setCurrentIndex(index);  // sets desired index, re-centers screen on new index
  ui.checkFollowNewest->setChecked(false);  // stops scrolling if search found


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
  QColor old_color = getButtonColor(widget);
  QColor color = QColorDialog::getColor(old_color, this);
  if (color.isValid()) {
    updateButtonColor(widget, color);
  }
}

QColor ConsoleWindow::getButtonColor(const QPushButton* button) const
{
  QString ss = button->styleSheet();
  QRegExp re("background: (#\\w*);");
  QColor old_color;
  if (re.indexIn(ss) >= 0) {
    old_color = QColor(re.cap(1));
  }
  return old_color;
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

void ConsoleWindow::loadColorButtonSetting(const QString& key, QPushButton* button)
{
  QSettings settings;
  QColor defaultColor;
  // The color buttons don't have a default value set in the .ui file, so we need to
  // supply defaults for them here in case the appropriate setting isn't found.
  if (button == ui.debugColorWidget) {
    defaultColor = Qt::gray;
  }
  else if (button == ui.infoColorWidget) {
    defaultColor = Qt::black;
  }
  else if (button == ui.warnColorWidget) {
    defaultColor = QColor(255, 127, 0);
  }
  else if (button == ui.errorColorWidget) {
    defaultColor = Qt::red;
  }
  else if (button == ui.fatalColorWidget) {
    defaultColor = Qt::magenta;
  }
  QColor color = settings.value(key, defaultColor).value<QColor>();
  updateButtonColor(button, color);
}

void ConsoleWindow::toggleAlternateRowColors(bool checked)
{
  ui.messageList->setAlternatingRowColors(checked);

  QSettings settings;
  settings.setValue(SettingsKeys::ALTERNATE_LOG_ROW_COLORS, checked);
}

void ConsoleWindow::loadSettings()
{
  // First, load all the boolean settings...
  loadBooleanSetting(SettingsKeys::DISPLAY_TIMESTAMPS, ui.action_ShowTimestamps);
  loadBooleanSetting(SettingsKeys::ABSOLUTE_TIMESTAMPS, ui.action_AbsoluteTimestamps);
  loadBooleanSetting(SettingsKeys::USE_REGEXPS, ui.action_RegularExpressions);
  loadBooleanSetting(SettingsKeys::COLORIZE_LOGS, ui.action_ColorizeLogs);
  loadBooleanSetting(SettingsKeys::FOLLOW_NEWEST, ui.checkFollowNewest);

  // The severity level has to be handled a little differently, since they're all combined
  // into a single integer mask under the hood.  First they have to be loaded from the settings,
  // then set in the UI, then the mask has to actually be applied.
  QSettings settings;
  bool showDebug = settings.value(SettingsKeys::SHOW_DEBUG, true).toBool();
  bool showInfo = settings.value(SettingsKeys::SHOW_INFO, true).toBool();
  bool showWarn = settings.value(SettingsKeys::SHOW_WARN, true).toBool();
  bool showError = settings.value(SettingsKeys::SHOW_ERROR, true).toBool();
  bool showFatal = settings.value(SettingsKeys::SHOW_FATAL, true).toBool();
  ui.checkDebug->setChecked(showDebug);
  ui.checkInfo->setChecked(showInfo);
  ui.checkWarn->setChecked(showWarn);
  ui.checkError->setChecked(showError);
  ui.checkFatal->setChecked(showFatal);
  setSeverityFilter();

  // Load button colors.
  loadColorButtonSetting(SettingsKeys::DEBUG_COLOR, ui.debugColorWidget);
  loadColorButtonSetting(SettingsKeys::INFO_COLOR, ui.infoColorWidget);
  loadColorButtonSetting(SettingsKeys::WARN_COLOR, ui.warnColorWidget);
  loadColorButtonSetting(SettingsKeys::ERROR_COLOR, ui.errorColorWidget);
  loadColorButtonSetting(SettingsKeys::FATAL_COLOR, ui.fatalColorWidget);

  // Finally, load the filter contents.
  QString includeFilter = settings.value(SettingsKeys::INCLUDE_FILTER, "").toString();
  ui.includeText->setText(includeFilter);
  QString excludeFilter = settings.value(SettingsKeys::EXCLUDE_FILTER, "").toString();
  ui.excludeText->setText(excludeFilter);

  bool alternate_row_colors = settings.value(SettingsKeys::ALTERNATE_LOG_ROW_COLORS, true).toBool();
  ui.messageList->setAlternatingRowColors(alternate_row_colors);
}
}  // namespace swri_console

