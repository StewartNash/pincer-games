/*
 * File: yolo_object_detector_node.cpp
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Node file for YOLO object detector
 *
 */

#include "darknet_emulator/YoloObjectDetector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	auto yoloObjectDetector = std::make_shared<darknet_emulator::YoloObjectDetector>();

	yoloObjectDetector->init();

	rclcpp::spin(yoloObjectDetector->get_node_base_interface());

	rclcpp::shutdown();

	return 0;
}
