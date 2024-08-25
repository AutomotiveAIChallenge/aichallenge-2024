#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <geometry_msgs/msg/point.hpp>

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
  }

private:
  // Callback function for PredictedObjects messages
  void objects_callback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received PredictedObjects message");

    // Initialize a new PredictedObjects message
    auto filtered_msg = autoware_auto_perception_msgs::msg::PredictedObjects();
    filtered_msg.header = msg->header; // Copy the header from the received message

    // Iterate through received objects and process them as needed
    for (const auto &object : msg->objects)
    {
      // Example filtering or processing (add your own logic here)
      filtered_msg.objects.push_back(object);
    }

    // Publish the filtered objects
    publisher_->publish(filtered_msg);
  }

  // Callback function for Float64MultiArray messages
  void data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received Float64MultiArray message");

    // Initialize a new PredictedObjects message
    auto predicted_objects_msg = autoware_auto_perception_msgs::msg::PredictedObjects();
    predicted_objects_msg.header.stamp = this->now();
    predicted_objects_msg.header.frame_id = "map";

    // Process the Float64MultiArray data
    for (size_t i = 0; i < msg->data.size(); i += 4)
    {
        if (i + 3 < msg->data.size()) // Ensure there's enough data
        {
            // Create a new PredictedObject
            autoware_auto_perception_msgs::msg::PredictedObject object;
            object.kinematics.initial_pose_with_covariance.pose.position.x = msg->data[i];
            object.kinematics.initial_pose_with_covariance.pose.position.y = msg->data[i + 1];
            object.kinematics.initial_pose_with_covariance.pose.position.z = msg->data[i + 2];

            // Add additional details like shape or classification as needed
            object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
            object.shape.dimensions.x = msg->data[i + 3] * 2; // Example: diameter -> dimensions
            object.shape.dimensions.y = msg->data[i + 3] * 2;
            object.shape.dimensions.z = msg->data[i + 3] * 2;

            autoware_auto_perception_msgs::msg::ObjectClassification classification;
            classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
            classification.probability = 1.0; // Assign a probability, e.g., 1.0 for certainty
            object.classification.push_back(classification);

            // Add the object to the PredictedObjects message
            predicted_objects_msg.objects.push_back(object);
        }
    }

    // Publish the PredictedObjects message
    publisher_->publish(predicted_objects_msg);
}


  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr objects_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr data_subscription_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeAndPublishNode>());
  rclcpp::shutdown();
  return 0;
}
