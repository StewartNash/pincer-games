/*
 * File: YoloObjectDetector.cpp
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Implementation of darknet
 * emulator.
 */

#include "darknet_emulator/YoloObjectDetector.hpp"

using namespace darknet_emulator;

YoloObjectDetector::YoloObjectDetector() : Node("darknet_emulator") {


}

YoloObjectDetector::~YoloObjectDetector() {

}

void YoloObjectDetector::init() {
	std::string boundingBoxesTopicName;
	int boundingBoxesQueueSize;
	bool boundingBoxesLatch;
	
	boundingBoxesTopicName = "bounding_boxes";
	boundingBoxesQueueSize = 10;
	boundingBoxesLatch = false;

	rclcpp::QoS bounding_boxes_publisher_qos(boundingBoxesQueueSize);
	if (boundingBoxesLatch) {
		bounding_boxes_publisher_qos.transient_local();
	}
	boundingBoxesPublisher_ = this->create_publisher<darknet_emulator_msgs::msg::BoundingBoxes>(boundingBoxesTopicName, bounding_boxes_publisher_qos);
}
