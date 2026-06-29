// Copyright 2024 Open Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PAI_DATA_COLLECTION__DATA_COLLECTOR_HPP_
#define PAI_DATA_COLLECTION__DATA_COLLECTOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "yaml-cpp/yaml.h"

namespace pai_data_collection
{

/**
 * @brief Data collector node for recording ROS2 topics to MCAP files
 *
 * This node reads topic configuration from a YAML file and records
 * specified topics to an MCAP bag file. The configuration file specifies:
 * - topics: list of topic names to record
 * - bag_path: output path for the MCAP file
 * - bag_name: name of the output MCAP file (without extension)
 */
class DataCollector : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit DataCollector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor - ensures proper cleanup of the writer
   */
  ~DataCollector();

private:
  /**
   * @brief Load configuration from YAML file
   * @param config_path Path to the configuration YAML file
   * @return True if configuration loaded successfully, false otherwise
   */
  bool load_config(const std::string & config_path);

  /**
   * @brief Initialize the rosbag2 writer
   * @return True if writer initialized successfully, false otherwise
   */
  bool initialize_writer();

  /**
   * @brief Generic topic callback for serialized messages
   * @param msg Shared pointer to the serialized message
   * @param topic_name Name of the topic the message was received on
   */
  void topic_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string & topic_name);

  /**
   * @brief Get the message type string from the ROS graph
   * @param topic_name Name of the topic
   * @return Message type string (e.g., "std_msgs/msg/String")
   */
  std::string get_message_type(const std::string & topic_name);

  // Configuration parameters
  std::vector<std::string> topics_to_record_;
  std::string bag_path_;
  std::string bag_name_;
  std::string storage_type_;

  // Rosbag2 writer
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

  // Storage for topic type mappings
  std::map<std::string, std::string> topic_types_;

  // Subscriptions storage
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

}  // namespace pai_data_collection

#endif  // PAI_DATA_COLLECTION__DATA_COLLECTOR_HPP_
