#include "arduino_comm/arduinocomm.hpp"

using namespace pincergames;

ArduinoComm::ArduinoComm() : rclcpp::Node("arduino_comms") {
	subscription_ = this->create_subscription<std_msgs::msg::String>("ui_command", 1, std::bind(&ArduinoComm::subscriberCallback, this, std::placeholders::_1));
}

ArduinoComm::~ArduinoComm() {

}

void ArduinoComm::subscriberCallback(const std_msgs::msg::String data) {

}
