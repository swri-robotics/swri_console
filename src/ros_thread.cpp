#include <QCoreApplication>
#include "include/swri_console/ros_thread.h"

using namespace swri_console;

RosThread::RosThread() :
  is_connected_(false),
  is_running_(true)
{
  int argc = QCoreApplication::argc();
  ros::init(argc, QCoreApplication::argv(), "swri_console",
            ros::init_options::AnonymousName |
            ros::init_options::NoRosout);
}

void RosThread::run()
{
  while (is_running_)
  {
    bool master_status = ros::master::check();

    if (!is_connected_ && master_status) {
      startRos();
    } else if (is_connected_ && !master_status) {
      stopRos();
    } else if (is_connected_ && master_status) {
      ros::spinOnce();
      Q_EMIT spun();
    }
    msleep(50);
  }
}


void RosThread::shutdown()
{
  is_running_ = false;
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void RosThread::startRos()
{
  ros::start();
  is_connected_ = true;

  ros::NodeHandle nh;
  rosout_sub_ = nh.subscribe("/rosout", 10000,
                             &RosThread::handleRosout,
                             this);
  Q_EMIT connected(true);
}

void RosThread::stopRos()
{
  ros::shutdown();
  is_connected_ = false;
  Q_EMIT connected(false);
}

void RosThread::handleRosout(const rosgraph_msgs::LogConstPtr &msg)
{
  Q_EMIT logReceived(msg);
}
