#include <cstdio>
#include "fz80_interfaces/msg/vehicle_attitude.hpp"
#include "rclcpp/rclcpp.hpp"


class reveive_hk_data : public rclcpp::Node
{
private:
  /* data */
  rclcpp::TimerBase::SharedPtr timer_main;
  rclcpp::Publisher<fz80_interfaces::msg::VehicleAttitude>::SharedPtr hk_data_publisher;

public:
  reveive_hk_data(std::string name);
  ~reveive_hk_data();

  void timer_main_callback();
};

reveive_hk_data::reveive_hk_data(std::string name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "Node %s is started.",name.c_str());
  timer_main = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&reveive_hk_data::timer_main_callback, this));
}

reveive_hk_data::~reveive_hk_data()
{
}

void reveive_hk_data::timer_main_callback()
{

}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  /*产生一个reveive_hk_data的节点*/
  auto node = std::make_shared<reveive_hk_data>("reveive_hk");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
