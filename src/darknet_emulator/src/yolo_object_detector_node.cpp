/*
 * File: yolo_object_detector_node.cpp
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Node file for YOLO object detector
 *
 */

#include "darknet_emulator/YoloObjectDetector.hpp"
#include "rclcpp/rclcpp.hpp"

/*
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	//auto yoloObjectDetector = std::make_shared<darknet_emulator::YoloObjectDetector>();
	//yoloObjectDetector->init();
	//rclcpp::spin(yoloObjectDetector->get_node_base_interface());
	rclcpp::spin(std::make_shared<darknet_emulator::YoloObjectDetector>());

	rclcpp::shutdown();

	return 0;
}
*/

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	auto yoloobjectdetector_node = std::make_shared<darknet_emulator::YoloObjectDetector>();
	//auto yoloemulator_node = yoloobjectdetector_node->getYoloEmulator();
	auto yoloemulator_node = yoloobjectdetector_node->myYoloEmulator;
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(yoloobjectdetector_node);
	executor.add_node(yoloemulator_node);
	executor.spin();

	rclcpp::shutdown();

	return 0;
}
