// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <swri_console/console_window.h>
#include <swri_console/log_database.h>
#include <swri_console/log_database_proxy_model.h>
#include <swri_console/node_list_model.h>
#include <swri_console/settings_keys.h>

#include <QColorDialog>
#include <QInputDialog>
#include <QRegularExpression>
#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QFileDialog>
#include <QDir>
#include <QScrollBar>
#include <QMenu>
#include <QSettings>
#include <QVariant>

using namespace Qt;

namespace log_level_mask {
  static constexpr uint8_t DEBUG = 1 << 0;
  static constexpr uint8_t INFO = 1 << 1;
  static constexpr uint8_t WARN = 1 << 2;
  static constexpr uint8_t ERROR = 1 << 3;
  static constexpr uint8_t FATAL = 1 << 4;
};

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

  QObject::connect(ui.action_Use_human_readable_time, SIGNAL(toggled(bool)),
                   db_proxy_, SLOT(setHumanReadableTime(bool)));

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

  QObject::connect(ui.action_RegularExpressions, SIGNAL(toggled(bool)),
                   this, SLOT(updateHighlightLabel()));

  QObject::connect(ui.action_SelectFont, SIGNAL(triggered(bool)),
                   this, SIGNAL(selectFont()));

  QObject::connect(ui.action_MessageFormat, SIGNAL(triggered(bool)),
                   this, SLOT(selectMessageFormat()));

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
  QObject::connect(ui.highlightColorWidget, SIGNAL(clicked(bool)),
                   this, SLOT(setHighlightColor()));

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

  QObject::connect(node_click_handler_, SIGNAL(nodeColorSelected(const std::string&, const QColor&)),
                    this, SLOT(setNodeColor(const std::string&, const QColor&)));
  QObject::connect(node_click_handler_, SIGNAL(nodeColorCleared(const std::string&)),
                    this, SLOT(clearNodeColor(const std::string&)));

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
    this, SLOT(excludeTextEdited()));
  QObject::connect(
    ui.excludeText, SIGNAL(cursorPositionChanged(int, int)),
    this, SLOT(excludeTextEdited()));
  QObject::connect(
    ui.excludeText, SIGNAL(editingFinished()),
    this, SLOT(excludeTextEdited()));
  QObject::connect(
    ui.action_RegularExpressions, SIGNAL(toggled(bool)),
    this, SLOT(excludeTextEdited()));

  QObject::connect(
    ui.highlightText, SIGNAL(textChanged(const QString &)),
    this, SLOT(highlightFilterUpdated(const QString &)));

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
    node_names[i] = node_names[i].split("/", Qt::SkipEmptyParts).last();
  }
    
  setWindowTitle(QString("SWRI Console (") + node_names.join(", ") + ")");
}

void ConsoleWindow::setSeverityFilter()
{
  uint8_t mask = 0;

  if (ui.checkDebug->isChecked()) {
    mask |= log_level_mask::DEBUG;
  }
  if (ui.checkInfo->isChecked()) {
    mask |= log_level_mask::INFO;
  }
  if (ui.checkWarn->isChecked()) {
    mask |= log_level_mask::WARN;
  }
  if (ui.checkError->isChecked()) {
    mask |= log_level_mask::ERROR;
  }
  if (ui.checkFatal->isChecked()) {
    mask |= log_level_mask::FATAL;
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
  QStringList items = text.split(";", Qt::SkipEmptyParts);
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

void ConsoleWindow::highlightFilterUpdated(const QString &text)
{
  // Unlike exclude, highlighting never hides anything, so there's no harm
  // in applying it live as the user types -- same as include.
  QStringList items = text.split(";", Qt::SkipEmptyParts);
  QStringList filtered;

  for (int i = 0; i < items.size(); i++) {
    QString x = items[i].trimmed();
    if (!x.isEmpty()) {
      filtered.append(x);
    }
  }

  db_proxy_->setHighlightFilters(filtered);
  db_proxy_->setHighlightRegexpPattern(text);
  updateHighlightLabel();
}

void ConsoleWindow::excludeTextEdited()
{
  QString text = ui.excludeText->text();
  bool editing = ui.excludeText->hasFocus();

  if (ui.action_RegularExpressions->isChecked()) {
    // There's no natural way to split a single regexp into "committed" and
    // "in progress" pieces, so treat the whole pattern as a preview while
    // the field has focus, and only commit it as an active filter once the
    // user moves on (matches the plain-string behavior below).
    if (editing) {
      db_proxy_->setExcludePreviewFilter(text);
    } else {
      db_proxy_->setExcludeRegexpPattern(text);
      db_proxy_->setExcludePreviewFilter(QString());
    }
  } else {
    QString pending;
    QString committedText = text;

    if (editing) {
      int cursor = ui.excludeText->cursorPosition();

      // QString::lastIndexOf(ch, -1) means "search from the end", so guard
      // the cursor == 0 case explicitly rather than passing cursor - 1 == -1
      // and getting a match from the wrong end of the string.
      int start = 0;
      if (cursor > 0) {
        int prevSemicolon = text.lastIndexOf(';', cursor - 1);
        start = (prevSemicolon == -1) ? 0 : prevSemicolon + 1;
      }

      int end = text.indexOf(';', cursor);
      if (end == -1) {
        end = text.length();
      }
      pending = text.mid(start, end - start).trimmed();
      committedText.remove(start, end - start);
    }

    QStringList items = committedText.split(";", Qt::SkipEmptyParts);
    QStringList filtered;

    for (int i = 0; i < items.size(); i++) {
      QString x = items[i].trimmed();
      if (!x.isEmpty()) {
        filtered.append(x);
      }
    }

    db_proxy_->setExcludeFilters(filtered);
    db_proxy_->setExcludePreviewFilter(pending);
  }

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

void ConsoleWindow::updateHighlightLabel()
{
  if (db_proxy_->isHighlightValid()) {
    ui.highlightLabel->setText("Highlight");
  } else {
    ui.highlightLabel->setText("<font color='red'>Highlight</font>");
  }
}

void ConsoleWindow::setFont(const QFont &font)
{
  ui.messageList->setFont(font);
  ui.nodeList->setFont(font);
}

void ConsoleWindow::selectMessageFormat()
{
  bool ok = false;
  QString format = QInputDialog::getText(
    this,
    tr("Message Format"),
    tr("Format string. Supported tokens: {severity} {name} {function_name}\n"
       "{file_name} {line_number} {time} {message}\n"
       "Leave blank to use the Show Timestamps/logger name/function name\n"
       "options instead."),
    QLineEdit::Normal,
    db_proxy_->outputFormat(),
    &ok);

  if (ok) {
    db_proxy_->setOutputFormat(format);
  }
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

void ConsoleWindow::setHighlightColor()
{
  chooseButtonColor(ui.highlightColorWidget);
}

void ConsoleWindow::setNodeColor(const std::string& node, const QColor& color)
{
  db_proxy_->setNodeColor(node, color);
}

void ConsoleWindow::clearNodeColor(const std::string& node)
{
  db_proxy_->clearNodeColor(node);
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
  QRegularExpression re("background: (#\\w*);");
  QColor old_color;
  QRegularExpressionMatch match = re.match(ss);
  if (match.hasMatch()) {
    old_color = QColor(match.captured(1));
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
  else if (widget == ui.highlightColorWidget) {
    db_proxy_->setHighlightColor(color);
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
    // PlaceholderText is Qt's own "dimmed but still legible" text role, so
    // it stays readable against the current theme's background without us
    // having to hand-pick a gray that only works for one theme.
    defaultColor = QApplication::palette().color(QPalette::PlaceholderText);
  }
  else if (button == ui.infoColorWidget) {
    defaultColor = QApplication::palette().color(QPalette::Text);
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
  else if (button == ui.highlightColorWidget) {
    // Highlight is the same role Qt uses for its own selection background,
    // so it's already guaranteed to read well against the current theme.
    defaultColor = QApplication::palette().color(QPalette::Highlight);
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
  loadBooleanSetting(SettingsKeys::HUMAN_READABLE_TIME, ui.action_Use_human_readable_time);
  loadBooleanSetting(SettingsKeys::USE_REGEXPS, ui.action_RegularExpressions);
  loadBooleanSetting(SettingsKeys::COLORIZE_LOGS, ui.action_ColorizeLogs);
  loadBooleanSetting(SettingsKeys::FOLLOW_NEWEST, ui.checkFollowNewest);

  // The severity level has to be handled a little differently, since they're all combined
  // into a single integer mask under the hood.  First they have to be loaded from the settings,
  // then set in the UI, then the mask has to actually be applied.
  QSettings settings;
  bool displayTimestamp = settings.value(SettingsKeys::DISPLAY_TIMESTAMPS, true).toBool();
  bool showDebug = settings.value(SettingsKeys::SHOW_DEBUG, true).toBool();
  bool showInfo = settings.value(SettingsKeys::SHOW_INFO, true).toBool();
  bool showWarn = settings.value(SettingsKeys::SHOW_WARN, true).toBool();
  bool showError = settings.value(SettingsKeys::SHOW_ERROR, true).toBool();
  bool showFatal = settings.value(SettingsKeys::SHOW_FATAL, true).toBool();
  ui.action_Use_human_readable_time->setEnabled(displayTimestamp);
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
  loadColorButtonSetting(SettingsKeys::HIGHLIGHT_COLOR, ui.highlightColorWidget);

  QVariantMap nodeColors = settings.value(SettingsKeys::NODE_COLORS).toMap();
  for (auto it = nodeColors.constBegin(); it != nodeColors.constEnd(); ++it) {
    db_proxy_->setNodeColor(it.key().toStdString(), it.value().value<QColor>());
  }

  // RCUTILS_CONSOLE_OUTPUT_FORMAT, if set, takes priority every launch (it's
  // meant to reflect the current shell environment, not a one-time default).
  // Otherwise fall back to whatever custom format the user last saved via
  // the Message Format dialog, or the legacy fixed layout if neither is set.
  QString envOutputFormat = QString::fromLocal8Bit(qgetenv("RCUTILS_CONSOLE_OUTPUT_FORMAT"));
  QString outputFormat = envOutputFormat.isEmpty()
    ? settings.value(SettingsKeys::OUTPUT_FORMAT, "").toString()
    : envOutputFormat;
  db_proxy_->setOutputFormat(outputFormat);

  // Finally, load the filter contents.
  QString includeFilter = settings.value(SettingsKeys::INCLUDE_FILTER, "").toString();
  ui.includeText->setText(includeFilter);
  QString excludeFilter = settings.value(SettingsKeys::EXCLUDE_FILTER, "").toString();
  ui.excludeText->setText(excludeFilter);
  QString highlightFilter = settings.value(SettingsKeys::HIGHLIGHT_FILTER, "").toString();
  ui.highlightText->setText(highlightFilter);

  bool alternate_row_colors = settings.value(SettingsKeys::ALTERNATE_LOG_ROW_COLORS, true).toBool();
  ui.messageList->setAlternatingRowColors(alternate_row_colors);
}
}  // namespace swri_console

