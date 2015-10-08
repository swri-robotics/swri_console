// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

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
