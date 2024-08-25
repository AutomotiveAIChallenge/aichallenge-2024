#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <random>

class MergeAndPublishNode : public rclcpp::Node
{
public:
  MergeAndPublishNode() : Node("merge_and_publish_node")
  {
    // Subscribe to the PredictedObjects topic
    objects_subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects", 10,
      std::bind(&MergeAndPublishNode::objects_callback, this, std::placeholders::_1));

    // Subscribe to the Float64MultiArray topic
    data_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/aichallenge/objects", 10,
      std::bind(&MergeAndPublishNode::data_callback, this, std::placeholders::_1));

    // Publisher to publish the filtered PredictedObjects
    publisher_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects_filtered", 10);

    // Set up a timer to periodically publish data
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // Adjust the period as needed
      std::bind(&MergeAndPublishNode::timer_callback, this));
  }

private:
  // Callback function for PredictedObjects messages
  void objects_callback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
  {
    // Store the latest PredictedObjects message
    last_objects_msg_ = msg;
  }

  // Callback function for Float64MultiArray messages
  void data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Store the latest Float64MultiArray message
    last_data_msg_ = msg;
  }

  // Function to generate a random UUID
  unique_identifier_msgs::msg::UUID generate_random_uuid()
  {
    unique_identifier_msgs::msg::UUID uuid;
    std::random_device rd;
    std::uniform_int_distribution<uint8_t> dist(0, 255);
    for (auto &byte : uuid.uuid) {
      byte = dist(rd);
    }
    return uuid;
  }

  // Timer callback to publish the combined data
  void timer_callback()
  {
    if (!last_objects_msg_ || !last_data_msg_)
    {
      // Skip if either message hasn't been received yet
      return;
    }

    // Initialize a new PredictedObjects message
    auto predicted_objects_msg = autoware_auto_perception_msgs::msg::PredictedObjects();
    predicted_objects_msg.header.stamp = this->now();
    predicted_objects_msg.header.frame_id = "map";

    // Process the Float64MultiArray data
    for (size_t i = 0; i < last_data_msg_->data.size(); i += 4)
    {
      if (i + 3 < last_data_msg_->data.size()) // Ensure there's enough data
      {
        // Create a new PredictedObject
        autoware_auto_perception_msgs::msg::PredictedObject object;
        object.object_id.uuid = generate_random_uuid().uuid; // Fix here
        object.existence_probability = 1.0; // Set existence probability to 100%

        object.kinematics.initial_pose_with_covariance.pose.position.x = last_data_msg_->data[i];
        object.kinematics.initial_pose_with_covariance.pose.position.y = last_data_msg_->data[i + 1];
        object.kinematics.initial_pose_with_covariance.pose.position.z = last_data_msg_->data[i + 2];

        // Add additional details like shape or classification as needed
        object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
        object.shape.dimensions.x = last_data_msg_->data[i + 3] * 2; // Example: diameter -> dimensions
        object.shape.dimensions.y = last_data_msg_->data[i + 3] * 2;
        object.shape.dimensions.z = last_data_msg_->data[i + 3] * 2;

        autoware_auto_perception_msgs::msg::ObjectClassification classification;
        classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
        classification.probability = 1.0; // Assign a probability, e.g., 1.0 for certainty
        object.classification.push_back(classification);

        // Add the object to the PredictedObjects message
        predicted_objects_msg.objects.push_back(object);
      }
    }

    // Merge with last received PredictedObjects message
    for (const auto &object : last_objects_msg_->objects)
    {
      auto new_object = object;
      new_object.object_id.uuid = generate_random_uuid().uuid; // Fix here
      new_object.existence_probability = 1.0; // Set existence probability to 100%
      predicted_objects_msg.objects.push_back(new_object);
    }

    // Publish the merged and filtered PredictedObjects message
    publisher_->publish(predicted_objects_msg);
  }

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr objects_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscription_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Store the last received messages
  autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr last_objects_msg_;
  std_msgs::msg::Float64MultiArray::SharedPtr last_data_msg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeAndPublishNode>());
  rclcpp::shutdown();
  return 0;
}
