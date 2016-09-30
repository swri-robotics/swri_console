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

#ifndef SWRI_CONSOLE_CONSOLE_WINDOW_H_
#define SWRI_CONSOLE_CONSOLE_WINDOW_H_

#include <QtWidgets/QMainWindow>
#include <QColor>
#include <QPushButton>
#include <QSettings>
#include "ui_console_window.h"

namespace swri_console
{
class LogDatabase;
class LogDatabaseProxyModel;
class NodeListModel;
class ConsoleWindow : public QMainWindow {
  Q_OBJECT
  
 public:
  ConsoleWindow(LogDatabase *db);
  ~ConsoleWindow();
  
  void closeEvent(QCloseEvent *event); // Overloaded function

 Q_SIGNALS:
  void createNewWindow();
  void readBagFile();
  void selectFont();
                                       
 public Q_SLOTS:
  void clearAll();
  void clearMessages();
  void saveLogs();
  void connected(bool);
  void setSeverityFilter();
  void nodeSelectionChanged();
  void messagesAdded();
  void showLogContextMenu(const QPoint& point);
  void selectAllLogs();
  void copyLogs();
  void copyExtendedLogs();
  void setFollowNewest(bool);
  void toggleAlternateRowColors(bool);
  
  void userScrolled(int);

  void includeFilterUpdated(const QString &);
  void excludeFilterUpdated(const QString &);
  void updateIncludeLabel();
  void updateExcludeLabel();

  void setFont(const QFont &font);

  void setDebugColor();
  void setInfoColor();
  void setWarnColor();
  void setErrorColor();
  void setFatalColor();

private:
  void chooseButtonColor(QPushButton* widget);
  QColor getButtonColor(const QPushButton* button) const;
  void updateButtonColor(QPushButton* widget, const QColor& color);

  template <typename T>
  void loadBooleanSetting(const QString& key, T* element){
    QSettings settings;
    bool val = settings.value(key, element->isChecked()).toBool();
    if (val != element->isChecked()) {
      element->setChecked(val);
    }
  };
  void loadColorButtonSetting(const QString& key, QPushButton* button);
  void loadSettings();

  Ui::ConsoleWindow ui;
  LogDatabase *db_;
  LogDatabaseProxyModel *db_proxy_;
  NodeListModel *node_list_model_;
};  // class ConsoleWindow
}  // namespace swri_console

#endif // SWRI_CONSOLE_CONSOLE_WINDOW_H_
