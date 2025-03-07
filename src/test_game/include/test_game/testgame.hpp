#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "darknet_emulator_msgs/msg/bounding_boxes.hpp"
#include "darknet_emulator_msgs/msg/bounding_box.hpp"

namespace pincergames {

class TestGame : public rclcpp::Node {
	public:
		TestGame();
		~TestGame();
	private:
		rclcpp::Publisher <std_msgs::msg::String>::SharedPtr publisher_;

		rclcpp::Subscription<darknet_emulator_msgs::msg::BoundingBoxes>::SharedPtr subscriber_;
		size_t count_;
		void callback(const darknet_emulator_msgs::msg::BoundingBoxes data);
};

} /* namespace pincergames */
