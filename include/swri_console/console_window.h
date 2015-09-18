#ifndef SWRI_CONSOLE_CONSOLE_WINDOW_H_
#define SWRI_CONSOLE_CONSOLE_WINDOW_H_

#include <QtGui/QMainWindow>
#include "ui_console_window.h"

namespace swri_console
{
class LogDatabase;
class LogDatabaseProxyModel;
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
  void clearLogs();
  void clearNodes();
  void connected(bool);
  void setSeverityFilter();
  void nodeSelectionChanged();
  void messagesAdded();

  void userScrolled(int);

  void includeFilterUpdated(const QString &);
  void excludeFilterUpdated(const QString &);
  void updateIncludeLabel();
  void updateExcludeLabel();

  void setFont(const QFont &font);
  
private:  
  Ui::ConsoleWindow ui;
  LogDatabase *db_;
  LogDatabaseProxyModel *db_proxy_;
};  // class ConsoleWindow
}  // namespace swri_console

#endif // SWRI_CONSOLE_CONSOLE_WINDOW_H_
