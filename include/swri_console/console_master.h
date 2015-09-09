#ifndef SWRI_CONSOLE_ROSOUT_SOURCE_H_
#define SWRI_CONSOLE_ROSOUT_SOURCE_H_

#include <ros/ros.h>
#include <string>
#include <QObject>
#include <QList>
#include <rosgraph_msgs/Log.h>
#include <swri_console/log_database.h>

namespace swri_console
{
typedef std::vector<rosgraph_msgs::LogConstPtr> MessageList;

class ConsoleWindow;
class ConsoleMaster : public QObject
{
  Q_OBJECT;

 public:  
  ConsoleMaster(int argc, char** argv );
  virtual ~ConsoleMaster();
  
  virtual void timerEvent(QTimerEvent *event);
                                             
 public Q_SLOTS:
  void createNewWindow();
  
 Q_SIGNALS:
  void connected(bool);
  void rosShutdown();

 private:
  void startRos();
  void stopRos();
  void processNewData();
  
  int update_timer_;
  
  bool connected_;
  ros::Subscriber rosout_sub_;
  void handleRosout(const rosgraph_msgs::LogConstPtr &msg);
  
  QList<ConsoleWindow*> windows_;

  LogDatabase db_;
};
}  // namespace swri_console

#endif  // SWRI_CONSOLE_ROSOUT_SOURCE_H_
