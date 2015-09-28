#ifndef SWRI_CONSOLE_BAG_READER_H
#define SWRI_CONSOLE_BAG_READER_H

#include <QObject>
#include <QString>
#include <QMetaType>

#include <rosgraph_msgs/Log.h>

namespace swri_console
{
  class BagReader : public QObject
  {
    Q_OBJECT
  public:
    /**
     * Reads a bag file at the specified path.  Any log messages that were broadcast on the
     * /rosout topic will be loaded and displayed.
     * @param[in] filename The name of the bag file to load.
     */
    void readBagFile(const QString& filename);

  public Q_SLOTS:
    /**
     * Displays a file dialog that prompts the user to pick a bag file.  After picking a bag
     * file, log messages in the bag will be read and displayed in the log console.
     */
    void promptForBagFile();

  Q_SIGNALS:

    /**
     * Emitted every time a log message is received.  This will likely be emitted several times
     * per bag file; finishedReading will be emitted when we're done.
     */
    void logReceived(const rosgraph_msgs::LogConstPtr& msg);

    /**
     * Emitted after we're completely done reading the bag file.
     */
    void finishedReading();
  };
}

#endif //SWRI_CONSOLE_BAG_READER_H
