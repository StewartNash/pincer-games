/*
 * File: YoloEmulator.hpp
 * Author: Stewart Nash
 * Date: Feburary 23, 2025
 * Description: Header for YoloEmulator which
 * will generate fake bounding boxed for the
 * darknet emulator.
 */
#pragma once

namespace darknet_emulator {

typedef struct {
	float x, y, w, h;
} box;

typedef struct detection {
	box bbox;
	int classes;
	float *prob;
	float *mask;
	float objectness;
	int sort_class;
} detection;

typedef struct network {

} network;

class YoloEmulator {
	public:
		YoloEmulator() {}
		~YoloEmulator();
		void draw_detections(detection *dets, int nboxes);
		void free_detections(detection *dets, int nboxes);
		network *load_network();
};

} /* namespace darknet_emulator */
