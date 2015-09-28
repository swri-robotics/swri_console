#include <QFileDialog>
#include <QDir>

#include "include/swri_console/bag_reader.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace swri_console;

void BagReader::readBagFile(const QString& filename)
{
  rosbag::Bag bag;
  bag.open(filename.toStdString(), rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/rosout"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View::const_iterator iter;

  for(iter = view.begin(); iter != view.end(); ++iter)
  {
    rosgraph_msgs::LogConstPtr log = iter->instantiate<rosgraph_msgs::Log>();
    if (log != NULL ) {
      emit logReceived(log);
    }
    else {
      qWarning("Got a message that was not a log message but a: %s", iter->getDataType().c_str());
    }
  }

  emit finishedReading();
}

void BagReader::promptForBagFile()
{
  QString filename = QFileDialog::getOpenFileName(NULL,
                                                  tr("Open Bag File"),
                                                  QDir::homePath(),
                                                  tr("Bag Files (*.bag)"));

  if (filename != NULL)
  {
    readBagFile(filename);
  }
}
