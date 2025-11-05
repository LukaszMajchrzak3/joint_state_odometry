#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomPrinter : public rclcpp::Node
{
public:
  OdomPrinter() : Node("odom_printer")
  {
    // Subskrypcja topicu /odom
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomPrinter::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Odom printer node started — listening to /odom");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &pos = msg->pose.pose.position;
    const auto &ori = msg->pose.pose.orientation;

    std::cout << std::fixed;
    std::cout.precision(4);
    std::cout << "x: " << pos.x
              << ", y: " << pos.y
              << ", z: " << pos.z
              << " | qx: " << ori.x
              << ", qy: " << ori.y
              << ", qz: " << ori.z
              << ", qw: " << ori.w
              << std::endl;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomPrinter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

