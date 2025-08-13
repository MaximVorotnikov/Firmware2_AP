#include <chrono>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include <random>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class CarDriverNode : public rclcpp::Node
{
public:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr field_vel_pub;

    void init(int id)
  {
      std::string local_namespace = "/vehicle_qr" + std::to_string(id);
      std::string topic = local_namespace + "/cmd_vel";
      field_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
  }

  CarDriverNode()
  : Node("car_driver_node")
  {
    auto timer_callback = [this]() -> void 
    {
      geometry_msgs::msg::Twist vel;
      random_device rd;
      mt19937 gen(rd());
      uniform_int_distribution<> dist(-5,5);

      vel.linear.x = dist(gen) / 10.0;
      vel.linear.y = dist(gen) / 10.0;

      this->field_vel_pub->publish(vel);
    
    };

    timer_ = this->create_wall_timer(3000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CarDriverNode>();

  node->declare_parameter("id", 1);

  int id = 0;
  int default_id = 2;

  node->get_parameter_or("id", id, default_id);

  node->init(id);


  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}