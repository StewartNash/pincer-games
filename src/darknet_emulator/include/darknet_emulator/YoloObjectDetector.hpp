/*
 * File: YoloObjectDetector.hpp
 * Author: Stewart Nash
 * Date: February 16, 2025
 * Description: Header for YoloObjectDetector
 * which will generate fake bounding boxes as
 * a darknet emulator.
 */
#pragma once

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"

#include "darknet_emulator_msgs/msg/bounding_boxes.hpp"
#include "darknet_emulator_msgs/msg/bounding_box.hpp"

#include "darknet_emulator/YoloEmulator.hpp"

namespace darknet_emulator {

// Bounding box of the detected object.
typedef struct {
	float x, y, w, h, prob;
	int num, Class;
} RosBox_;

class YoloObjectDetector : public rclcpp::Node {
	//friend class YoloEmulator;
	public:
		explicit YoloObjectDetector();
		~YoloObjectDetector();
		void init();
		
		static const int MAXIMUM_BOXES = 512;
	private:
		void timerCallback();
		bool readParameters();
		rclcpp::TimerBase::SharedPtr timer_;		
		rclcpp::Publisher<darknet_emulator_msgs::msg::BoundingBoxes>::SharedPtr boundingBoxesPublisher_;
		
		darknet_emulator_msgs::msg::BoundingBoxes boundingBoxesResults_;
		int demoClasses_;
		RosBox_ *roiBoxes_;
		network *net_;
		
		void *detectInThread();
		void *detectLoop(void *ptr);
		void setupNetwork();
		void yolo();
		void *publishInThread();
		
		YoloEmulator myYoloEmulator = YoloEmulator();
};

} /* namespace darknet_emulator */
