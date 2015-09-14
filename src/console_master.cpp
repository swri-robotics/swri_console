#include <swri_console/console_master.h>
#include <swri_console/console_window.h>
#include <QFontDialog>

namespace swri_console
{
ConsoleMaster::ConsoleMaster()
  :
  connected_(false),
  window_font_(QFont("Ubuntu Mono", 9))
{
  // The RosThread takes advantage of queued connections when emitting log messages
  // to ensure that the messages are processed in the console window's event thread.
  // In order for that to work, we have to manually register the message type with
  // Qt's QMetaType system.
  qRegisterMetaType<rosgraph_msgs::LogConstPtr>("rosgraph_msgs::LogConstPtr");
}

ConsoleMaster::~ConsoleMaster()
{
  ros_thread_.shutdown();
  ros_thread_.wait();
}

void ConsoleMaster::createNewWindow()
{
  ConsoleWindow* win = new ConsoleWindow(&db_);
  windows_.append(win);

  win->setFont(window_font_);
  QObject::connect(win, SIGNAL(createNewWindow()),
                   this, SLOT(createNewWindow()));

  QObject::connect(&ros_thread_, SIGNAL(connected(bool)),
                   win, SLOT(connected(bool)));

  QObject::connect(this,
                   SIGNAL(fontChanged(const QFont &)),
                   win, SLOT(setFont(const QFont &)));

  QObject::connect(win, SIGNAL(selectFont()),
                   this, SLOT(selectFont()));

  QObject::connect(&ros_thread_, SIGNAL(logReceived(const rosgraph_msgs::LogConstPtr& )),
                   &db_, SLOT(queueMessage(const rosgraph_msgs::LogConstPtr&) ));

  QObject::connect(&ros_thread_, SIGNAL(spun()),
                   &db_, SLOT(processQueue()));

  if (!ros_thread_.isRunning())
  {
    ros_thread_.start();
  }

  win->show();
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
