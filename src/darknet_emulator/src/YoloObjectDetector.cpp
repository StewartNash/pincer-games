/*
 * File: YoloObjectDetector.cpp
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Implementation of darknet
 * emulator.
 */
//TODO: Resolve threading issues (Implement threading)
#include "darknet_emulator/YoloObjectDetector.hpp"

using namespace darknet_emulator;

YoloObjectDetector::YoloObjectDetector() : Node("darknet_emulator"), publishTimer(0) {
	roiBoxes_ = new RosBox_[MAXIMUM_BOXES];
	frameWidth_ = myYoloEmulator.X_PIXELS;
	frameHeight_ = myYoloEmulator.Y_PIXELS;	
}

YoloObjectDetector::~YoloObjectDetector() {
	delete[] roiBoxes_;
}

void YoloObjectDetector::init() {
	std::string objectDetectorTopicName;
	int objectDetectorQueueSize;
	bool objectDetectorLatch;
	std::string boundingBoxesTopicName;
	int boundingBoxesQueueSize;
	bool boundingBoxesLatch;
	
	objectDetectorTopicName = "object_detector";
	objectDetectorQueueSize = 10;
	objectDetectorLatch = false;
	boundingBoxesTopicName = "bounding_boxes";
	boundingBoxesQueueSize = 10;
	boundingBoxesLatch = false;

	rclcpp::QoS object_publisher_qos(objectDetectorQueueSize);
	if (objectDetectorLatch) {
		object_publisher_qos.transient_local();
	}
	objectPublisher_ = this->create_publisher<darknet_emulator_msgs::msg::ObjectCount>(objectDetectorTopicName, object_publisher_qos);
	
	rclcpp::QoS bounding_boxes_publisher_qos(boundingBoxesQueueSize);
	if (boundingBoxesLatch) {
		bounding_boxes_publisher_qos.transient_local();
	}
	boundingBoxesPublisher_ = this->create_publisher<darknet_emulator_msgs::msg::BoundingBoxes>(boundingBoxesTopicName, bounding_boxes_publisher_qos);
	timer_ = this->create_wall_timer(std::literals::chrono_literals::operator""ms(500), std::bind(&YoloObjectDetector::timerCallback, this));
	std::string str1(1, myYoloEmulator.HUMAN_CHARACTER);
	std::string str2(1, myYoloEmulator.ROBOT_CHARACTER);
	classLabels_ = {str1, str2};
	numClasses_ = myYoloEmulator.NUMBER_OF_CLASSES;
	rosBoxes_ = std::vector<std::vector<RosBox_>>(numClasses_);
	rosBoxCounter_ = std::vector<int>(numClasses_);
	
	detectLoop(nullptr);
}

void YoloObjectDetector::timerCallback() {
	myYoloEmulator.incrementTime(0.5);
	if (++publishTimer == PUBLISHING_PERIOD) {
		publishTimer = 0;
		yolo();
	}
}

void *YoloObjectDetector::detectInThread() {
	detection *dets;
	int nboxes;

	dets = nullptr;
	nboxes = 0;

	myYoloEmulator.draw_detections(dets, nboxes, demoClasses_);
	if (nboxes * demoClasses_ > MAXIMUM_BOXES) {
		throw "Maximum box count exceeded.";
	}
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
	while (!stopDetection) {
		detectInThread();
	}
	
	return ptr;
}

void YoloObjectDetector::setupNetwork() {
	net_ = myYoloEmulator.load_network();
}

void YoloObjectDetector::yolo() {
	//roiBoxes_ = new RosBox_[MAXIMUM_BOXES];
	if (!demoDone_) {
		publishInThread();
	}
}

void *YoloObjectDetector::publishInThread() {
	int num = roiBoxes_[0].num;
	if (num > 0 && num <= 100) {
		for (int i = 0; i < num; i++) {
			for (int j = 0; j < numClasses_; j++) {
				if (roiBoxes_[i].Class == j) {
					rosBoxes_[j].push_back(roiBoxes_[i]);
					rosBoxCounter_[j]++;
				}
			}
		}
		
		darknet_emulator_msgs::msg::ObjectCount msg;
		msg.header.stamp = this->now();
		msg.header.frame_id = "detection";
		msg.count = num;
		objectPublisher_->publish(msg);
		
		for (int i = 0; i < numClasses_; i++) {
			if (rosBoxCounter_[i] > 0) {
				darknet_emulator_msgs::msg::BoundingBox boundingBox;
				for (int j = 0; j < rosBoxCounter_[i]; j++) {
					int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
					int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
					int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
					int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;
					boundingBox.class_id = classLabels_[i];
					boundingBox.id = i;
					boundingBox.probability = rosBoxes_[i][j].prob;
					boundingBox.xmin = xmin;
					boundingBox.ymin = ymin;
					boundingBox.xmax = xmax;
					boundingBox.ymax = ymax;
					boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
				}
			}
		}
		boundingBoxesResults_.header.stamp = this->now();
		boundingBoxesResults_.header.frame_id = "detection";
		boundingBoxesPublisher_->publish(boundingBoxesResults_);
	}
	boundingBoxesResults_.bounding_boxes.clear();
	for (int i = 0; i < numClasses_; i++) {
		rosBoxes_[i].clear();
		rosBoxCounter_[i] = 0;
	}
	
	return 0;
}
