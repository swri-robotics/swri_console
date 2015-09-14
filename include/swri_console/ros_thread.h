#ifndef SWRI_CONSOLE_ROS_THREAD_H
#define SWRI_CONSOLE_ROS_THREAD_H

#include <QThread>

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <QMetaType>

#include <boost/shared_ptr.hpp>

namespace swri_console
{
  class RosThread : public QThread
  {
    Q_OBJECT
  public:
    RosThread();
    /*
     * Shuts down ROS and causes the thread to exit.
     */
    void shutdown();

  Q_SIGNALS:
    /**
     * Emitted every time we are successfully connected to or disconnected from ROS.
     */
    void connected(bool);
    /**
     * Emitted every time a log message is received.  This can be emitted multiple times per spin of
     * the ROS core; wait until spun() is emitted to do any processing on them.
     */
    void logReceived(const rosgraph_msgs::LogConstPtr &msg);
    /**
     * Emitted after every time ros::spinOnce() completes.
     */
    void spun();

  protected:
    void run();

  private:
    void handleRosout(const rosgraph_msgs::LogConstPtr &msg);
    void startRos();
    void stopRos();

    volatile bool is_running_;
    bool is_connected_;
    ros::Subscriber rosout_sub_;
    boost::shared_ptr<ros::NodeHandle> nh_;
  };
}

Q_DECLARE_METATYPE(rosgraph_msgs::LogConstPtr);

#endif //SWRI_CONSOLE_ROS_THREAD_H
