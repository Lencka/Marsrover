#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher() : Node("odometry_publisher")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdometryPublisher::publish_odometry, this));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void publish_odometry()
  {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.header.stamp = this->now();

    // Set position and orientation
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    msg.pose.pose.orientation = tf2::toMsg(orientation);

    // Set velocity and acceleration
    msg.twist.twist.linear.x = 0.1;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    publisher_->publish(msg);

    // Publish the transform
    geometry_msgs::msg::TransformStamped odom_transform;
    odom_transform.header.stamp = this->now();
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";
    odom_transform.transform.translation.x = msg.pose.pose.position.x;
    odom_transform.transform.translation.y = msg.pose.pose.position.y;
    odom_transform.transform.translation.z = msg.pose.pose.position.z;
    odom_transform.transform.rotation = msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_transform);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
