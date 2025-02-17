/*
 * File: YoloObjectDetector.hpp
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Header for YoloObjectDetector
 * which will generate fake bounding boxes as
 * a darknet emulator.
 */
#pragma once

//#include <chrono>
//#include <functional>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"

#include "darknet_emulator_msgs/msg/bounding_boxes.hpp"
#include "darknet_emulator_msgs/msg/bounding_box.hpp"

namespace darknet_emulator {

class YoloObjectDetector : public rclcpp::Node {
	public:
		explicit YoloObjectDetector();
		~YoloObjectDetector();
		void init();
	private:
		bool readParameters();
		
		rclcpp::Publisher<darknet_emulator_msgs::msg::BoundingBoxes>::SharedPtr boundingBoxesPublisher_;
		
		darknet_emulator_msgs::msg::BoundingBoxes boundingBoxesResults_;
};

}
