#include "arduino_comm/arduinocomm.hpp"

using namespace pincergames;

ArduinoComm::ArduinoComm() : rclcpp::Node("arduino_comm_node") {
	RCLCPP_INFO(this->get_logger(), "Starting ArduinoComm...");
	try {
		serial_port_.setPort("/dev/ttyUSB0");
		serial_port_.setBaudrate(9600);
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		serial_port_.setTimeout(timeout);
		serial_port_.open();
		
		if (serial_port_.isOpen()) {
			RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
		}
	} catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Exception opening serial port: %s", e.what());
		throw;
	}

	subscription_ = this->create_subscription<std_msgs::msg::String>("ui_command", 1, std::bind(&ArduinoComm::subscriberCallback, this, std::placeholders::_1));
}

ArduinoComm::~ArduinoComm() {
	if (serial_port_.isOpen()) {
		serial_port_.close();
		RCLCPP_INFO(this->get_logger(), "Serial port closed.");
	}
}

void ArduinoComm::subscriberCallback(const std_msgs::msg::String::SharedPtr message) {
	std::string command = message->data;
	RCLCPP_INFO(this->get_logger(), "Received command: '%s'", command.c_str());
	
	if (serial_port_.isOpen()) {
		serial_port_.write(command + "\n");
		RCLCPP_INFO(this->get_logger(), "Command sent to Arduino: %s", command.c_str());
	} else {
		RCLCPP_WARN(this->get_logger(), "Serial port is not open. Command not sent.");
	}
}
