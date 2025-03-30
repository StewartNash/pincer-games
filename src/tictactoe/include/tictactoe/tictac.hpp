#pragma once

//#include <chrono>
//#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <map>
#include <array>
#include <cstring>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "darknet_emulator_msgs/msg/bounding_boxes.hpp"
#include "darknet_emulator_msgs/msg/bounding_box.hpp"

namespace pincergames {

const std::string ROBOT_WINS = "\
  ____       *           *              _           \n\
 |  * \\ *** | |**   ___ | |_  __      *(*)_ **  **_ \n\
 | |_) / * \\| '* \\ / * \\| *_| \\ \\ /\\ / / | '_ \\/ __|\n\
 |  * < (*) | |_) | (_) | |_   \\ V  V /| | | | \\__ \\\n\
 |_| \\_\\___/|_.__/ \\___/ \\__|   \\_/\\_/ |_|_| |_|___/\
";

const std::string HUMAN_WINS = "\
  *   *                                        _           \n\
 | | | |_   * ***_**   ** ***** ***   __      *(*)_ **  **_ \n\
 | |_| | | | | '_ `  \\ / ` | '_ \\  \\ \\ /\\ / / | '_ \\/ __|\n\
 |  *  | |*| | | | | | | (_| | | | |  \\ V  V /| | | | \\__ \\\n\
 |_| |_|\\__,_|_| |_| |_|\\__,_|_| |_|   \\_/\\_/ |_|_| |_|___/\
";

const std::string TIE_GAME = "\
  ___ *   *               *   *      \n\
 |_ *| |*( )___    __ *  | |*(_) ___ \n\
  | || **|// **|  / *` | | *_| |/ _ \\\n\
  | || |_  \\__ \\ | (_| | | |_| |  __/\n\
 |___|\\__| |___/  \\__,_|  \\__|_|\\___|\
";

class TicTacToe : public rclcpp::Node {
	public:
		static constexpr int NUMBER_OF_POSITIONS = 19;
		static constexpr int BOARD_POSITIONS = 9;
		static constexpr int BOARD_SIZE = 9;
		static constexpr int CENTER_POSITION = 4;
		static constexpr int X_SIZE = 3;
		static constexpr int Y_SIZE = 3;
		
		TicTacToe();
		~TicTacToe();

		char boardState[BOARD_POSITIONS]; // Current visual state
		char committedMoves[BOARD_POSITIONS]; // Memory of committed moves
		char lastDetectedBoard[BOARD_POSITIONS]; // Store last detected board state
		
		char currentTurn; // X is robot and O is human
		short currentFigure; // Track which figure to use next (1 - 5)		
		bool gameActive;
		bool waitingForHuman;
		
		std::map<std::string, std::array<std::array<double, 3>, 2>> positions;
	private:
		//void timer_callback();
		//rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commandPublisher_;
		rclcpp::Subscription<darknet_emulator_msgs::msg::BoundingBoxes>::SharedPtr boundingBoxesSubscriber_;
		size_t count_;
		
		std::thread inputThread; // Create keyboard input thread for reset
		std::atomic<bool> stopThread{false};
		
		void handleKeyboardInput();
		void resetGame();
		int findBestMove();
		void boundingBoxesCallback(const darknet_emulator_msgs::msg::BoundingBoxes data);
		//double* applyJointOffsets(double coords[3]);
		//double** applyJointOffsets(double coords[3][2]);
		std::array<double, 3> applyJointOffsets(std::array<double, 3> coords);
		std::array<std::array<double, 3>, 2> applyJointOffsets(std::array<std::array<double, 3>, 2> coords);
		void clearTerminal();
		int mapBoundingBoxToGrid(darknet_emulator_msgs::msg::BoundingBox box);
		bool checkWinner();
		void displayBoard();
		void moveToPosition(std::string positionKey);
		void pickAndPlace(int targetPosition);
		void moveToCoordinates(double coords[3]);
		void moveToCoordinates(double coords[3][2]);
		void moveToCoordinates(std::array<double, 3> coords);
		void moveToCoordinates(std::array<std::array<double, 3>, 2> coords);
		std::string updateGripperState(std::string action);
		void updateDisplay(std::string status);
		
		static void printArray(char* array, int length);
};

} /* namespace pincergames */
