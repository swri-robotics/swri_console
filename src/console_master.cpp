#include <swri_console/console_master.h>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <swri_console/console_window.h>
#include <QFontDialog>

namespace swri_console
{
ConsoleMaster::ConsoleMaster(int argc, char** argv)
  :
  connected_(false),
  window_font_(QFont("Ubuntu Mono", 9))
{
  ros::init(argc, argv, "swri_console",
            ros::init_options::AnonymousName |
            ros::init_options::NoRosout);
  createNewWindow();

  update_timer_ = startTimer(100);
}

ConsoleMaster::~ConsoleMaster()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  // wait();
}

void ConsoleMaster::createNewWindow()
{
  ConsoleWindow* win = new ConsoleWindow(&db_);
  windows_.append(win);

  win->setFont(window_font_);
  QObject::connect(win, SIGNAL(createNewWindow()),
                   this, SLOT(createNewWindow()));

  QObject::connect(this, SIGNAL(connected(bool)),
                   win, SLOT(connected(bool)));

  QObject::connect(this,
                   SIGNAL(fontChanged(const QFont &)),
                   win, SLOT(setFont(const QFont &)));

  QObject::connect(win, SIGNAL(selectFont()),
                   this, SLOT(selectFont()));
                     
  win->show();
}

void ConsoleMaster::timerEvent(QTimerEvent *event)
{
  bool master_status = ros::master::check();
  
  if (!connected_ && master_status) {
    startRos();
  } else if (connected_ && !master_status) {
    stopRos();
  } else if (connected_ && master_status) {
    ros::spinOnce();
    db_.processQueue();
  }
}

void ConsoleMaster::startRos()
{
  ros::start();
  connected_ = true;

  ros::NodeHandle nh;
  rosout_sub_ = nh.subscribe("/rosout", 10000,
                             &ConsoleMaster::handleRosout,
                             this);
  ros::NodeHandle();
  Q_EMIT connected(true);
}

void ConsoleMaster::stopRos()
{
  ros::shutdown();
  connected_ = false;
  Q_EMIT connected(false);    
}

void ConsoleMaster::handleRosout(const rosgraph_msgs::LogConstPtr &msg)
{
  db_.queueMessage(*msg);
}

void ConsoleMaster::fontSelectionChanged(const QFont &font)
{
  window_font_ = font;
  Q_EMIT fontChanged(window_font_);
}

void ConsoleMaster::selectFont()
{
  QFont starting_font = window_font_;

  QFontDialog dlg(window_font_);
    
  QObject::connect(&dlg, SIGNAL(currentFontChanged(const QFont &)),
                   this, SLOT(fontSelectionChanged(const QFont &)));

  int ret = dlg.exec();

  if (ret == QDialog::Accepted) {
    if (window_font_ != dlg.selectedFont()) {
      window_font_ = dlg.selectedFont();
      Q_EMIT fontChanged(window_font_);
    }
  } else {
    if (window_font_ != starting_font) {
      window_font_ = starting_font;
      Q_EMIT fontChanged(window_font_);
    }
  }
}
}  // namespace swri_console
