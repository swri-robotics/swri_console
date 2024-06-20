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
#include <QMessageBox>

#include "swri_console/bag_reader.h"
#include <rosbag2_transport/reader_writer_factory.hpp>

using namespace swri_console;

void BagReader::readBagFile(const QString& filename)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = filename.toStdString();
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options);

  while (reader->has_next()) {
    auto msg = reader->read_next();

    if (msg->topic_name != "/rosout") {
      continue;
    }

    // Deserialize the message
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto log = std::make_shared<rcl_interfaces::msg::Log>();
    rclcpp::Serialization<rcl_interfaces::msg::Log> serialization_;
    serialization_.deserialize_message(&serialized_msg, log.get());

    emit logReceived(log);
  }

  emit finishedReading();
}

void BagReader::promptForBagFile()
{
  QString filename = QFileDialog::getOpenFileName(nullptr,
                                                  tr("Open Bag File"),
                                                  QDir::homePath(),
                                                  tr("Bag Files (*.mcap)"));

  if (filename != nullptr)
  {
    readBagFile(filename);
  }
}
