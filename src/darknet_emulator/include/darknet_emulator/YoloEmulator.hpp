/*
 * File: YoloEmulator.hpp
 * Author: Stewart Nash
 * Date: Feburary 23, 2025
 * Description: Header for YoloEmulator which
 * will generate fake bounding boxed for the
 * darknet emulator.
 */
#pragma once

#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <random>
//#include <unistd.h>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

class RobotState {
	public:
		RobotState();
		
		bool isGripperOpen;
		int gripperOpenCount;
		int gripperCloseCount;
		bool isPlacementReady;
	
		std::tuple<double, double, double> position;
		std::tuple<double, double, double> positionQueue;
		
		void processCommand(std::string command);
		//std::tuple<double, double, double> getPlacement();
};

class YoloEmulator : public rclcpp::Node {
	public:
		YoloEmulator();
		~YoloEmulator();
		
		static const int X_SIZE = 3;
		static const int Y_SIZE = 3;
		static const int BOARD_SIZE = 9;
		
		static const int X_PIXELS = 720;
		static const int Y_PIXELS = 480;
		
		// Grid boundaries in pixel coordinates
		static const int LEFT_BOUNDARY = 197;
		static const int RIGHT_BOUNDARY = 598;
		static const int TOP_BOUNDARY = 25;
		static const int BOTTOM_BOUNDARY = 431;
		
		static constexpr double FLOAT_LEFT_BOUNDARY = 197.0;
		static constexpr double FLOAT_RIGHT_BOUNDARY = 598.0;
		static constexpr double FLOAT_TOP_BOUNDARY = 25.0;
		static constexpr double FLOAT_BOTTOM_BOUNDARY = 431.0;		
		
		//static constexpr double X_DISTANCE_PIXELS = 100.25;
		static constexpr double X_DISTANCE_PIXELS = 133.667;
		//static constexpr double Y_DISTANCE_PIXELS = 101.5;
		static constexpr double Y_DISTANCE_PIXELS = 135.333;
		//static constexpr double MEAN_X_SEPARATION = 0.01;
		//static constexpr double MEAN_Y_SEPARATION = 0.01;
		static constexpr double X_DISTANCE = 0.213;
		static constexpr double Y_DISTANCE = 0.165;
		static constexpr double X_STDDEV = 0.032;
		static constexpr double Y_STDDEV = 0.048;
		static constexpr double X_ORIGIN = -1.42;
		static constexpr double Y_ORIGIN = 1.21;
		
		static constexpr double X_RESOLUTION = X_DISTANCE / X_DISTANCE_PIXELS;
		static constexpr double Y_RESOLUTION = Y_DISTANCE / Y_DISTANCE_PIXELS;
		
		static const char ROBOT_CHARACTER = 'X';
		static const char HUMAN_CHARACTER = 'O';
		
		static const int X_CLASS = 1;
		static const int O_CLASS = 0;
		static const int NUMBER_OF_CLASSES = 2;
		
		void incrementTime(double increment); // increment in seconds
		void draw_detections(detection*& dets, int& nboxes, int& classes);
		void free_detections(detection*& dets, int& nboxes);
		network *load_network();
		
		static std::vector<std::string> split(const std::string& s, char separator);
	private:
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
		size_t count_;
		
		RobotState myRobotState = RobotState();

		static std::map<int, std::tuple<double, double>> centerPositions;
		static std::map<int, std::tuple<int, int>> centerPixels;
		static std::map<int, std::tuple<int, int>> arrayLocations;
		
		//double averageXMoveTime = 15.0; // seconds
		//double averageOMoveTime = 15.0; // seconds
		//double xMoveTimeStdDev = 3.0; // seconds
		//double yMoveTimeStdDev = 3.0; // seconds
		double averageHumanMoveTime = 7.0; // seconds
		double averageRobotMoveTime = 7.0; // seconds
		double humanMoveTimeStdDev = 1.5; // seconds
		double robotMoveTimeStdDev = 1.5; // seconds
		double robotMoveTime;
		double humanMoveTime;
		double currentTime;
		double deltaTime;
		char boardState[3][3];
		double boardXPositions[3][3];
		double boardYPositions[3][3];
		//bool isXTurn;
		bool isHumanTurn;
		bool isMoveReceived; // Robot move
		int robotQueue;
		
		std::mt19937 generator;
		std::normal_distribution<double> robotTimeDistribution;
		std::normal_distribution<double> humanTimeDistribution;		

		void callback(std_msgs::msg::String command);
		bool checkEndGame();
		int findLocation(std::tuple<int, int> indices);
		void makeMove();
		void populateBoard(int location, char player);
		void printBoardState();
		void receiveMove(double xPosition, double yPosition);
		
		static int convertToLocation(int xPixel, int yPixel);
		static int convertToLocation(std::tuple<int, int> pixel);
		static int convertToLocation(double xPosition, double yPosition);
		static std::tuple<int, int> convertToPixels(double xPosition, double yPosition);
		static double getDistance(double x1, double x2);
		static double getDistance(int x1, int x2);
		static double getDistance(double x1, double y1, double x2, double y2);
		static double getDistance(int x1, int y1, int x2, int y2);
		static double getDistance(std::tuple<int, int> a, std::tuple<int, int> b);
		//static double getDistance(std::tuple<double, double> a, std::tuple<double, double> b);
};

} /* namespace darknet_emulator */
