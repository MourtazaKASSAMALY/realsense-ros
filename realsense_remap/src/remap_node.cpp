#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

/* -----------------------------------------------------------------*/
/* This example creates a subclass of Node and implements a remapping of incoming and outcoming odometry topics as well as tf */

class RemapNode : public rclcpp::Node
{
  public:
    RemapNode()
    : Node("remap_node")
    {
      topic_odom_in_ = declare_parameter("topic_odom_in", rclcpp::ParameterValue("/camera/odom/sample")).get<rclcpp::PARAMETER_STRING>();
      topic_odom_out_ = declare_parameter("topic_odom_out", rclcpp::ParameterValue("/odom")).get<rclcpp::PARAMETER_STRING>();
      wheels_topic_ = declare_parameter("wheels_topic", rclcpp::ParameterValue("/wheels_odom")).get<rclcpp::PARAMETER_STRING>();
      frame_id_out_ = declare_parameter("frame_id_out", rclcpp::ParameterValue("odom")).get<rclcpp::PARAMETER_STRING>();
      child_frame_id_out_ = declare_parameter("child_frame_id_out", rclcpp::ParameterValue("realsense")).get<rclcpp::PARAMETER_STRING>();

      subscription_realsense_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_odom_in_, 10, std::bind(&RemapNode::realsense_callback, this, _1));
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom_out_, 10);
      subscription_wheels_ = this->create_subscription<nav_msgs::msg::Odometry>(wheels_topic_, 10, std::bind(&RemapNode::wheels_callback, this, _1));

      std::shared_ptr<rclcpp::Node> nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      RCLCPP_INFO(this->get_logger(), "Remap node started");
    }

  private:
    void wheels_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if (realsense_odom == NULL) { return; }

      odom = *realsense_odom;
      odom.header.frame_id = frame_id_out_;
      odom.child_frame_id = child_frame_id_out_;
      odom.header.stamp = msg->header.stamp;

      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.transform.translation.x = odom.pose.pose.position.x;
      odom_tf.transform.translation.y = odom.pose.pose.position.y;
      odom_tf.transform.translation.z = odom.pose.pose.position.z;
      odom_tf.transform.rotation = odom.pose.pose.orientation;

      odom_tf.header.frame_id = frame_id_out_;
      odom_tf.child_frame_id = child_frame_id_out_;
      odom_tf.header.stamp = msg->header.stamp;

      publisher_->publish(odom);
      tf_broadcaster_->sendTransform(odom_tf);
    }

    void realsense_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      realsense_odom = msg;
    }

    nav_msgs::msg::Odometry::SharedPtr realsense_odom;
    nav_msgs::msg::Odometry odom;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_realsense_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_wheels_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    std::string topic_odom_in_;
    std::string topic_odom_out_;
    std::string wheels_topic_;
    std::string frame_id_out_;
    std::string child_frame_id_out_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemapNode>());
  rclcpp::shutdown();
  return 0;
}
