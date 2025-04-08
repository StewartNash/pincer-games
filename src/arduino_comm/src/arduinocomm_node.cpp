#include "arduino_comm/arduinocomm.hpp"

using namespace pincergames;

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArduinoComm>());
	rclcpp::shutdown();

	return 0;
}
