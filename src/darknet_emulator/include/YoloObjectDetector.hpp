/*
 * File: YoloObjectDetector.h
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Header for YoloObjectDetector
 * which will generate fake bounding boxes as
 * a darknet emulator.
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace darknet_emulator {

class YoloObjectDetector : public rclcpp::Node {
	public:
		explicit YoloObjectDetector();
		~YoloObjectDetector();
		void init();
	private:
		bool readParameters();
};

}
