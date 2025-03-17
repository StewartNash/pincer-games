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

//TODO: Review rows, columns, and indices to ensure correctness
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

YoloEmulator::YoloEmulator() :
  rclcpp::Node("yolo_emulator"),
  count_(0),
  generator(std::random_device{}()),
  humanTimeDistribution(averageHumanMoveTime, humanMoveTimeStdDev),
  robotTimeDistribution(averageRobotMoveTime, robotMoveTimeStdDev) {
	subscription_ = this->create_subscription<std_msgs::msg::String>("ui_command", 10, std::bind(&YoloEmulator::callback, this, std::placeholders::_1));
	robotMoveTime = robotTimeDistribution(generator);
	humanMoveTime = humanTimeDistribution(generator);
	isHumanTurn = true;
	isMoveReceived = false;
	currentTime = 0;
	deltaTime = 0;
}

YoloEmulator::~YoloEmulator() {

}

void YoloEmulator::incrementTime(double increment) {
	currentTime += increment;
	deltaTime += increment;
	
	if (isHumanTurn) {
		//sleep(humanMoveTime);
		if (deltaTime > humanMoveTime) {
			deltaTime = std::fmod(deltaTime, humanMoveTime);
			makeMove();
			isHumanTurn = false;
			humanMoveTime = humanTimeDistribution(generator);
		}
	} else {
		//sleep(robotMoveTime);
		if (isMoveReceived) {
			if (deltaTime > robotMoveTime) {
				deltaTime = std::fmod(deltaTime, robotMoveTime);
				populateBoard(robotQueue, ROBOT_CHARACTER);
				isMoveReceived = false;
				isHumanTurn = true;
				robotMoveTime = robotTimeDistribution(generator);
			}
		}
	}
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

void YoloEmulator::draw_detections(detection*& dets, int& nboxes, int& classes) {
	int numberOfBoxes;
	std::vector<detection> temporary;
	
	free_detections(dets, nboxes);
	numberOfBoxes = 0;
	for (int i = 0; i < X_SIZE; i++) {
		for (int j = 0; j < Y_SIZE; j++) {
			if (!boardState[j][i] == '\0') {
				++numberOfBoxes;
			}
		}
	}
	dets = new detection[numberOfBoxes];
	nboxes = 0;
	for (int i = 0; i < X_SIZE; i++) {
		for (int j = 0; j < Y_SIZE; j++) {
			if (!(boardState[j][i] == '\0'))  {
				int location = findLocation(std::make_tuple(j, i));
				if (boardState[j][i] == 'X') {
					dets[nboxes].classes = X_CLASS;
					dets[nboxes].prob = new float[NUMBER_OF_CLASSES];
					dets[nboxes].prob[X_CLASS] = 1.0;
					dets[nboxes].prob[O_CLASS] = 0.0;
					dets[nboxes].mask = new float[NUMBER_OF_CLASSES];
					dets[nboxes].mask[X_CLASS] = 1.0;
					dets[nboxes].mask[O_CLASS] = 0.0;					
				} else {
					dets[nboxes].classes = O_CLASS;
					dets[nboxes].prob = new float[NUMBER_OF_CLASSES];
					dets[nboxes].prob[X_CLASS] = 0.0;
					dets[nboxes].prob[O_CLASS] = 1.0;
					dets[nboxes].mask = new float[NUMBER_OF_CLASSES];
					dets[nboxes].mask[X_CLASS] = 0.0;
					dets[nboxes].mask[O_CLASS] = 1.0;
				}
				dets[nboxes].bbox.x = std::get<0>(centerPixels[location]);
				dets[nboxes].bbox.y = std::get<1>(centerPixels[location]);
				dets[nboxes].bbox.w = X_DISTANCE_PIXELS;
				dets[nboxes].bbox.h = Y_DISTANCE_PIXELS;
				dets[nboxes].objectness = 1.0;
				dets[nboxes].sort_class = 0;
				++nboxes;
			}
		}
	}
	classes = NUMBER_OF_CLASSES;	
}

void YoloEmulator::free_detections(detection*& dets, int& nboxes) {
	if (!(dets == nullptr)) {
		for (int i = 0; i < nboxes; i++) {
			delete[] dets[i].prob;
			delete[] dets[i].mask;
		}
		delete[] dets;
		dets = nullptr;
		nboxes = 0;
	}
}

network *YoloEmulator::load_network() {
	return nullptr;
}

bool YoloEmulator::checkEndGame() {
	char currentBoard[BOARD_SIZE];
	
	for (int i = 0; i < X_SIZE; i++) {
		for (int j = 0; j < Y_SIZE; j++) {
			currentBoard[i * Y_SIZE + j] = boardState[j][i];
		}
	}
			
	// Check for a winner in any direction
	std::vector<std::tuple<int, int, int>> winConditions = {
		{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, // Rows
		{0, 3, 6}, {1, 4, 7}, {2, 5, 8}, // Columns
		{0, 4, 8}, {2, 4, 6} // Diagonals
	};

	for (const auto& condition : winConditions) {
		int a = std::get<0>(condition);
		int b = std::get<1>(condition);
		int c = std::get<2>(condition);

		if (currentBoard[a] == currentBoard[b] &&
			currentBoard[b] == currentBoard[c] &&
			currentBoard[c] != '\0') {
			return true;
		}
	}
	
	bool isFull = true;
	for (int i = 0; i < BOARD_SIZE; i++) {
		if (currentBoard[i] == '\0') {
			isFull = false;
		}
	}
	
	if (isFull) {
		return true;
	}
	
	return false;
}

int YoloEmulator::findLocation(std::tuple<int, int> indices) {
	for (const auto& pair : arrayLocations) {
		if (pair.second == indices) {
			return pair.first;
			//break;
		}
	}
	
	return -1;
}

void YoloEmulator::makeMove() {
	int location;

	if (!checkEndGame()) {
		//TODO: Make random human move
		for (int i = 0; i < X_SIZE; i++) {
			for (int j = 0; j < Y_SIZE; j++) {
				if (boardState[j][i] == '\0') {
					location = findLocation(std::make_tuple(i, j));
					break;
				}
			}
		}
		populateBoard(location, HUMAN_CHARACTER);
	} else {
		// Game Over
	}
}

void YoloEmulator::populateBoard(int location, char player) {
	int row, column;
	
	row = std::get<0>(arrayLocations[location]);
	column = std::get<1>(arrayLocations[location]);
	//TODO: Check for conflicts
	boardState[row][column] = player;
}

void YoloEmulator::receiveMove(double xPosition, double yPosition) {
	if (!isHumanTurn && !isMoveReceived) {
		robotQueue = convertToLocation(convertToPixels(xPosition, yPosition));
		isMoveReceived = true;
	} else {
		throw "Multiple and/or out-of-turn robot moves received.";
	}	
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
	distance = std::sqrt(distance);
	
	return distance;
}

double YoloEmulator::getDistance(int x, int y) {
	return getDistance(static_cast<double>(x), static_cast<double>(y));
}

double YoloEmulator::getDistance(double x1, double y1, double x2, double y2) {
	double distance;
	
	distance = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	distance = std::sqrt(distance);;;
	
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
