

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pincergames {

class ArduinoComm : public rclcpp::Node {
	public:
		ArduinoComm();
		~ArduinoComm();
	private:
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
		void subscriberCallback(const std_msgs::msg::String data);
};

} /* namespace pincergames */
