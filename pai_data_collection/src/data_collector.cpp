#include "pai_data_collection/data_collector.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp" 
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rmw/rmw.h"

namespace pai_data_collection
{

DataCollector::DataCollector(const rclcpp::NodeOptions & options)
: rclcpp::Node("data_collector", options)
{
  // Declare parameters with defaults
  declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>());
  declare_parameter<std::string>("bag_path", "/tmp/ros_bags");
  declare_parameter<std::string>("bag_name", "recording_session");
  declare_parameter<std::string>("storage_type", "mcap");

  // Get parameter values from ROS 2 parameter system
  topics_to_record_ = get_parameter("topics").as_string_array();
  bag_path_ = get_parameter("bag_path").as_string();
  bag_name_ = get_parameter("bag_name").as_string();
  storage_type_ = get_parameter("storage_type").as_string();

  // Validate required parameters
  if (topics_to_record_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No topics specified in 'topics' parameter. "
      "Use --ros-args -p topics:='[/topic1,/topic2]' or set via params file");
    throw std::runtime_error("No topics specified");
  }

  // Initialize the rosbag2 writer
  if (!initialize_writer()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize rosbag2 writer");
    throw std::runtime_error("Failed to initialize writer");
  }

  RCLCPP_INFO(
    this->get_logger(),
    "DataCollector initialized. Recording %zu topics to %s/%s.mcap",
    topics_to_record_.size(), bag_path_.c_str(), bag_name_.c_str());
}

DataCollector::~DataCollector()
{
  // Clean up subscriptions
  subscriptions_.clear();

  // Close the writer
  if (writer_) {
    writer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "DataCollector shut down");
}

bool DataCollector::initialize_writer()
{
  try {
    // Create the writer
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    // Prepare storage options for MCAP format
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_ + "/" + bag_name_;
    storage_options.storage_id = storage_type_;
    storage_options.max_cache_size = 100000000UL;  // 100MB cache

    // Prepare converter options
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = rmw_get_serialization_format();
    converter_options.output_serialization_format = rmw_get_serialization_format();

    // Open the writer
    writer_->open(storage_options, converter_options);

    RCLCPP_INFO(
      this->get_logger(),
      "Rosbag2 writer initialized with storage ID: %s",
      storage_options.storage_id.c_str());

    // Subscribe to topics with a small delay to allow discovery
    auto subscribe_timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        // Get all currently available topics
        auto topic_names_and_types = this->get_topic_names_and_types();

        // Subscribe to configured topics that are available
        for (const auto & topic_name : topics_to_record_) {
          // Check if we already subscribed to this topic
          if (subscriptions_.find(topic_name) != subscriptions_.end()) {
            continue;
          }

          // Check if this topic is available in the system
          auto it = std::find_if(
            topic_names_and_types.begin(),
            topic_names_and_types.end(),
            [&topic_name](const auto & pair) { return pair.first == topic_name; }
          );

          if (it != topic_names_and_types.end()) {
            // Found the topic - create subscription
            const auto & topic_type = it->second[0];  // Get the first (usually only) type

            auto callback = [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
              this->topic_callback(msg, topic_name);
            };

            auto subscription = this->create_generic_subscription(
              topic_name,
              topic_type,
              rclcpp::QoS(rclcpp::SensorDataQoS()),
              callback);

            subscriptions_[topic_name] = subscription;
            topic_types_[topic_name] = topic_type;

            RCLCPP_INFO(
              this->get_logger(),
              "Subscribed to topic: %s (type: %s)",
              topic_name.c_str(), topic_type.c_str());

            // Register topic with the writer
            rosbag2_storage::TopicMetadata topic_metadata;
            topic_metadata.name = topic_name;
            topic_metadata.type = topic_type;
            topic_metadata.serialization_format = rmw_get_serialization_format();

            try {
              writer_->create_topic(topic_metadata);
              RCLCPP_INFO(
                this->get_logger(),
                "Created topic in bag: %s", topic_name.c_str());
            } catch (const std::exception & e) {
              RCLCPP_WARN(
                this->get_logger(),
                "Could not create topic in bag (might already exist): %s",
                e.what());
            }
          }
        }

        // Check if all topics have been subscribed to
        if (subscriptions_.size() == topics_to_record_.size()) {
          RCLCPP_INFO(
            this->get_logger(),
            "All configured topics have been subscribed to successfully");
          // Stop the timer - we're done discovering
          // Note: We'll keep spinning to continue receiving messages
        }
      });

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error initializing rosbag2 writer: %s", e.what());
    return false;
  }
}

void DataCollector::topic_callback(
  std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic_name)
{
  if (!writer_) {
    return;
  }

  try {
    // Create a bag message
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    // Copy serialized data
    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t * data) {
        auto fini_return = rcutils_uint8_array_fini(data);
        delete data;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to finalize serialized message: %s",
            rcutils_get_error_string().str);
        }
      });

    *bag_message->serialized_data = msg->release_rcl_serialized_message();
    bag_message->topic_name = topic_name;

    // Get current time as timestamp
    if (rcutils_system_time_now(&bag_message->recv_timestamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error getting current time: %s",
        rcutils_get_error_string().str);
      return;
    }

    // Write the message to the bag
    writer_->write(bag_message);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error writing message to bag: %s", e.what());
  }
}

std::string DataCollector::get_message_type(const std::string & topic_name)
{
  auto topic_names_and_types = this->get_topic_names_and_types();

  for (const auto & [name, types] : topic_names_and_types) {
    if (name == topic_name && !types.empty()) {
      return types[0];
    }
  }

  return "";
}

}  // namespace pai_data_collection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pai_data_collection::DataCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
