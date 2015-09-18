#ifndef SWRI_CONSOLE_CONSOLE_WINDOW_H_
#define SWRI_CONSOLE_CONSOLE_WINDOW_H_

#include <QtGui/QMainWindow>
#include <QPushButton>
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

  void setDebugColor();
  void setInfoColor();
  void setWarnColor();
  void setErrorColor();
  void setFatalColor();

private:
  void chooseButtonColor(QPushButton* widget);
  void updateButtonColor(QPushButton* widget, const QColor& color);

  Ui::ConsoleWindow ui;
  LogDatabase *db_;
  LogDatabaseProxyModel *db_proxy_;
};  // class ConsoleWindow
}  // namespace swri_console

#endif // SWRI_CONSOLE_CONSOLE_WINDOW_H_
