#ifndef SWRI_CONSOLE_ROSOUT_SOURCE_H_
#define SWRI_CONSOLE_ROSOUT_SOURCE_H_

#include <string>
#include <QObject>
#include <QList>
#include <QFont>
#include <rosgraph_msgs/Log.h>
#include <swri_console/log_database.h>

#include "ros_thread.h"

namespace swri_console
{
typedef std::vector<rosgraph_msgs::LogConstPtr> MessageList;

class ConsoleWindow;
class ConsoleMaster : public QObject
{
  Q_OBJECT;

 public:  
  ConsoleMaster();
  virtual ~ConsoleMaster();

 public Q_SLOTS:
  void createNewWindow();
  void fontSelectionChanged(const QFont &font);
  void selectFont();

 Q_SIGNALS:
  void fontChanged(const QFont &font);

 private:

  // All ROS operations are done on a separate thread to ensure they do not
  // cause the GUI thread to block.
  RosThread ros_thread_;
  
  bool connected_;

  QList<ConsoleWindow*> windows_;

  LogDatabase db_;

  QFont window_font_;
};
}  // namespace swri_console

#endif  // SWRI_CONSOLE_ROSOUT_SOURCE_H_
