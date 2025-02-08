#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <map>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
		
		TicTacToe();
		~TicTacToe();

		char boardState[9]; // Current visual state
		char committedMoves[9]; // Memory of committed moves
		char lastDetectedBoard[9]; // Store last detected board state
		
		char currentTurn; // X is robot and O is human
		short currentFigure; // Track which figure to use next (1 - 5)		
		bool gameActive;
		bool waitingForHuman;
		
		std::map<std::string, std::array<std::array<double, 3>, NUMBER_OF_POSITIONS>> positions;
	private:
		void timer_callback();
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		size_t count_;
		
		std::thread inputThread; // Create keyboard input thread for reset
		std::atomic<bool> stopThread{false};
		
		void handleKeyboardInput();
		void resetGame();
		void clearTerminal();
		void displayBoard();
};

}
