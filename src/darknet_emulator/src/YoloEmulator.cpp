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

std::map<int, std::tuple<double, double>> YoloEmulator::centerPositions = {
        {1, {-0.99, 1.21}},
	{2, {-0.99, 1.37}},
	{3, {-0.99, 1.54}},
	{4, {-1.20, 1.21}},
	{5, {-1.20, 1.37}},
	{6, {-1.20, 1.54}},
	{7, {-1.42, 1.21}},
	{8, {-1.42, 1.37}},
	{9, {-1.42, 1.54}}
};
	
std::map<int, std::tuple<int, int>> YoloEmulator::centerPixels = {
	{1, {297, 127}},
	{2, {297, 228}},
	{3, {297, 330}},
	{4, {398, 127}},
	{5, {398, 228}},
	{6, {398, 330}},
	{7, {498, 127}},
	{8, {498, 228}},
	{9, {498, 330}}
};
	
std::map<int, std::tuple<int, int>> YoloEmulator::arrayLocations = {
	{1, {0, 0}},
	{2, {0, 1}},
	{3, {0, 2}},
	{4, {1, 0}},
	{5, {1, 1}},
	{6, {1, 2}},
	{7, {2, 0}},
	{8, {2, 1}},
	{9, {2, 2}}
};

YoloEmulator::YoloEmulator() : rclcpp::Node("yolo_emulator"), count_(0) {
	subscription_ = this->create_subscription<std_msgs::msg::String>("ui_command", 10, std::bind(&YoloEmulator::callback, this, std::placeholders::_1));
/*
	////centerPositions[1] = std::make_tuple(-1.310, 1.197, -0.764);
	////centerPositions[2] = std::make_tuple(-1.096, 1.183, -0.716);
	////centerPositions[3] = std::make_tuple(-0.916, 1.171, -0.607);
	////centerPositions[4] = std::make_tuple(-0.766, 1.173, -0.431);
	////centerPositions[5] = std::make_tuple(-0.668, 1.199, -0.274);
	//centerPositions[1] = std::make_tuple(-0.888, 1.247, -0.016);
	//centerPositions[2] = std::make_tuple(-1.004, 1.373, 0.363);
	//centerPositions[3] = std::make_tuple(-1.080, 1.595, 0.925);
	//centerPositions[4] = std::make_tuple(-1.087, 1.193, -0.234);
	//centerPositions[5] = std::make_tuple(-1.164, 1.289, 0.169);
	//centerPositions[6] = std::make_tuple(-1.221, 1.471, 0.639);
	//centerPositions[7] = std::make_tuple(-1.314, 1.176, -0.339);
	//centerPositions[8] = std::make_tuple(-1.370, 1.258, 0.045);
	//centerPositions[9] = std::make_tuple(-1.387, 1.429, 0.520);
	
	//centerPositions[1] = std::make_tuple(-0.888, 1.247);
	//centerPositions[2] = std::make_tuple(-1.004, 1.373);
	//centerPositions[3] = std::make_tuple(-1.080, 1.595);
	//centerPositions[4] = std::make_tuple(-1.087, 1.193);
	//centerPositions[5] = std::make_tuple(-1.164, 1.289);
	//centerPositions[6] = std::make_tuple(-1.221, 1.471);
	//centerPositions[7] = std::make_tuple(-1.314, 1.176);
	//centerPositions[8] = std::make_tuple(-1.370, 1.258);
	//centerPositions[9] = std::make_tuple(-1.387, 1.429);
	
	centerPositions[1] = std::make_tuple(-0.99, 1.21);
	centerPositions[2] = std::make_tuple(-0.99, 1.37);
	centerPositions[3] = std::make_tuple(-0.99, 1.54);
	centerPositions[4] = std::make_tuple(-1.20, 1.21);
	centerPositions[5] = std::make_tuple(-1.20, 1.37);
	centerPositions[6] = std::make_tuple(-1.20, 1.54);
	centerPositions[7] = std::make_tuple(-1.42, 1.21);
	centerPositions[8] = std::make_tuple(-1.42, 1.37);
	centerPositions[9] = std::make_tuple(-1.42, 1.54);
	
	centerPixels[1] = std::make_tuple(297, 127);
	centerPixels[2] = std::make_tuple(297, 228);
	centerPixels[3] = std::make_tuple(297, 330);
	centerPixels[4] = std::make_tuple(398, 127);
	centerPixels[5] = std::make_tuple(398, 228);
	centerPixels[6] = std::make_tuple(398, 330);
	centerPixels[7] = std::make_tuple(498, 127);
	centerPixels[8] = std::make_tuple(498, 228);
	centerPixels[9] = std::make_tuple(498, 330);
	
	arrayLocations[1] = std::make_tuple(0, 0);
	arrayLocations[2] = std::make_tuple(0, 1);
	arrayLocations[3] = std::make_tuple(0, 2);
	arrayLocations[4] = std::make_tuple(1, 0);
	arrayLocations[5] = std::make_tuple(1, 1);
	arrayLocations[6] = std::make_tuple(1, 2);
	arrayLocations[7] = std::make_tuple(2, 0);
	arrayLocations[8] = std::make_tuple(2, 1);
	arrayLocations[9] = std::make_tuple(2, 2);
*/
}

YoloEmulator::~YoloEmulator() {

}

void YoloEmulator::callback(std_msgs::msg::String command) {
	std::string data;
	std::vector<std::string> partition;
	double x, y, z;

	data = command.data;
	partition = split(data, ',');
	try {
		x = std::stod(partition[0]);
		y = std::stod(partition[1]);
		z = std::stod(partition[2]);
		receiveMove(x, y);
	} catch (const std::invalid_argument& e) {
		std::cerr << "Error: Invalid argument - " << e.what() << std::endl;
	} catch (const std::out_of_range& e) {
		std::cerr << "Error: Out of range - " << e.what() << std::endl;
	}
}

void YoloEmulator::draw_detections(detection *dets, int nboxes) {
	dets = nullptr;
}

void YoloEmulator::free_detections(detection *dets, int nboxes) {

}

network *YoloEmulator::load_network() {
	return nullptr;
}

void YoloEmulator::makeMove() {

}

void YoloEmulator::populateBoard(int location, char player) {
	int row, column;
	
	row = std::get<0>(arrayLocations[location]);
	column = std::get<1>(arrayLocations[location]);
	boardState[row][column] = player;
}

void YoloEmulator::receiveMove(double xPosition, double yPosition) {
	int location;
	
	location = convertToLocation(convertToPixels(xPosition, yPosition));
	populateBoard(location, ROBOT_CHARACTER);
}

// Static methods

int YoloEmulator::convertToLocation(int xPixel, int yPixel) {
	double minimumDistance;
	bool is_first;
	std::tuple<int, int> pixel;
	int location;
	
	pixel = std::make_tuple(xPixel, yPixel);	
	is_first = true;
	for (int i = 0; i < BOARD_SIZE; i++)  {
		if (is_first) {
			location = i + 1;
			minimumDistance = getDistance(pixel, centerPixels[i + 1]);
			is_first = false;
		} else {
			if (getDistance(pixel, centerPixels[i + 1]) < minimumDistance) {
				location = i + 1;
				minimumDistance = getDistance(pixel, centerPixels[i + 1]);
		  	}
		}
	}

	return location;
}

int YoloEmulator::convertToLocation(std::tuple<int, int> pixel) {
	return convertToLocation(std::get<0>(pixel), std::get<1>(pixel));
}

int YoloEmulator::convertToLocation(double xPosition, double yPosition) {
	std::tuple<int, int> output;
	
	output = convertToPixels(xPosition, yPosition);
	
	return convertToLocation(std::get<0>(output), std::get<1>(output));
}

std::tuple<int, int> YoloEmulator::convertToPixels(double x, double y) {
	std::tuple<int, int> output;
	int xPixel, yPixel;
	double xOriginPixel, yOriginPixel;
	double xPixelSpace, yPixelSpace;
	
	xPixelSpace = (FLOAT_RIGHT_BOUNDARY - FLOAT_LEFT_BOUNDARY) / 4.0;
	yPixelSpace = (FLOAT_BOTTOM_BOUNDARY - FLOAT_BOTTOM_BOUNDARY) / 4.0;
	xOriginPixel = std::get<0>(centerPixels[1]);
	yOriginPixel = std::get<1>(centerPixels[1]);
	xPixel = (x - X_ORIGIN) * (xPixelSpace / X_DISTANCE) + xOriginPixel;
	yPixel = (y - Y_ORIGIN) * (yPixelSpace / Y_DISTANCE) + yOriginPixel;
	output = std::make_tuple(xPixel, yPixel);
	
	return output;
}

double YoloEmulator::getDistance(double x, double y) {
	double distance;
	
	distance = (x - y) * (x - y);
	//TODO: Take square root of distance	
	
	return distance;
}

double YoloEmulator::getDistance(int x, int y) {
	return getDistance(static_cast<double>(x), static_cast<double>(y));
}

double YoloEmulator::getDistance(double x1, double y1, double x2, double y2) {
	double distance;
	
	distance = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	//TODO: Take square root of distance
	
	return distance;
}

double YoloEmulator::getDistance(int x1, int y1, int x2, int y2) {
	return getDistance(static_cast<double>(x1),
		static_cast<double>(y1),
		static_cast<double>(x2),
		static_cast<double>(y2));
}

double YoloEmulator::getDistance(std::tuple<int, int> a, std::tuple<int, int> b) {
	return getDistance(std::get<0>(a),
		std::get<0>(b),
		std::get<1>(a),
		std::get<1>(b));
}

/*
double getDistance(std::tuple<double, double> a, std::tuple<double, double> b) {
	return getDistance(std::get<0>(a),
	 	std::get<0>(b),
	 	std::get<1>(a),
	 	std::get<1>(b));
}
*/

std::vector<std::string> YoloEmulator::split(const std::string& s, char separator) {
	std::vector<std::string> output;

	std::string::size_type prev_pos = 0, pos = 0;

	while((pos = s.find(separator, pos)) != std::string::npos) {
		std::string substring( s.substr(prev_pos, pos - prev_pos) );
		output.push_back(substring);
		prev_pos = ++pos;
	}

	output.push_back(s.substr(prev_pos, pos - prev_pos)); // Last word

	return output;
}
