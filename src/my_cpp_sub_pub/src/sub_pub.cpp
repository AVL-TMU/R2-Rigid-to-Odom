#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class SubPubNode : public rclcpp::Node
{
public:
  SubPubNode()
  : Node("sub_pub_node")
  {

    subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "rigid_bodies", 10, std::bind(&SubPubNode::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
  }

void topic_callback(const mocap4r2_msgs::msg::RigidBodies::UniquePtr msg) const
  {
    auto odom_msg = px4_msgs::msg::VehicleOdometry();

    odom_msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    

    if (!msg->rigidbodies.empty()) {
      odom_msg.position[0] = msg->rigidbodies[0].pose.position.x;
      odom_msg.position[1] = msg->rigidbodies[0].pose.position.y;
      odom_msg.position[2] = msg->rigidbodies[0].pose.position.z;
    }

    RCLCPP_INFO(this->get_logger(), "Received RigidBodies message with %zu rigid bodies", msg->rigidbodies.size());
    for (size_t i = 0; i < msg->rigidbodies.size(); ++i) {
      const auto& rigid_body = msg->rigidbodies[i];
      RCLCPP_INFO(this->get_logger(), "RigidBody %zu: position=(%f, %f, %f)", i, rigid_body.pose.position.x, rigid_body.pose.position.y, rigid_body.pose.position.z);
    }
    publisher_->publish(odom_msg);
  }

private:

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubPubNode>());
  rclcpp::shutdown();
  return 0;
}