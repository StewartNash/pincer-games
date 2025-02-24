/*
 * File: YoloEmulator.cpp
 * Author: Stewart Nash
 * Date: February 23, 2025
 * Description: YoloEmulator which will
 * generate fake bounding boxes for the
 * darknet emulator.
 */

#include "darknet_emulator/YoloEmulator.hpp"

using namespace darknet_emulator;

YoloEmulator::~YoloEmulator() {

}

void YoloEmulator::draw_detections(detection *dets, int nboxes) {
	dets = nullptr;
}

void YoloEmulator::free_detections(detection *dets, int nboxes) {

}

network *YoloEmulator::load_network() {
	return nullptr;
}
