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

void *YoloObjectDetector::detectInThread() {
	detection *dets;
	int nboxes;

	dets = nullptr;
	nboxes = 0;

	myYoloEmulator.draw_detections(dets, nboxes);
	// extract the bounding boxes and send them to ROS
	int i, j;
	int count = 0;
	for (i = 0; i < nboxes; ++i) {
		float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
		float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
		float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
		float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

		if (xmin < 0)
			xmin = 0;
		if (ymin < 0)
			ymin = 0;
		if (xmax > 1)
			xmax = 1;
		if (ymax > 1)
			ymax = 1;

		// iterate through possible boxes and collect the bounding boxes
		for (j = 0; j < demoClasses_; ++j) {
			if (dets[i].prob[j]) {
				float x_center = (xmin + xmax) / 2;
				float y_center = (ymin + ymax) / 2;
				float BoundingBox_width = xmax - xmin;
				float BoundingBox_height = ymax - ymin;

				// define bounding box
				// BoundingBox must be 1% size of frame (3.2x2.4 pixels)
				if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
					roiBoxes_[count].x = x_center;
					roiBoxes_[count].y = y_center;
					roiBoxes_[count].w = BoundingBox_width;
					roiBoxes_[count].h = BoundingBox_height;
					roiBoxes_[count].Class = j;
					roiBoxes_[count].prob = dets[i].prob[j];
					count++;
				}
			}
		}
	}

	// create array to store found bounding boxes
	// if no object detected, make sure that ROS knows that num = 0
	if (count == 0) {
		roiBoxes_[0].num = 0;
	} else {
		roiBoxes_[0].num = count;
	}

	myYoloEmulator.free_detections(dets, nboxes);

	return 0;
}

void *YoloObjectDetector::detectLoop(void *ptr) {

}

void YoloObjectDetector::setupNetwork() {
	net_ = myYoloEmulator.load_network();
}

void YoloObjectDetector::yolo() {

}

void *YoloObjectDetector::publishInThread() {

}
