#include <cstdio>
#include "fz80_interfaces/msg/vehicle_attitude.hpp"
#include "rclcpp/rclcpp.hpp"


class reveive_hk_data : public rclcpp::Node
{
private:
  /* data */
public:
  reveive_hk_data(std::string name);
  ~reveive_hk_data();
};

reveive_hk_data::reveive_hk_data(std::string name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "Node %s is started.",name.c_str());
}

reveive_hk_data::~reveive_hk_data()
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
