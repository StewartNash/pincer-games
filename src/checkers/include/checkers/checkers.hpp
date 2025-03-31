#pragma once

#include <atomic>
#include <map>
#include <array>
#include <cstring>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "darknet_emulator_msgs/msg/bounding_boxes.hpp"
#include "darknet_emulator_msgs/msg/bounding_box.hpp"

namespace pincergames {

const std::string ROBOT_WINS = "\
Robot wins\
";

const std::string HUMAN_WINS = "\
Human wins\
";

const std::string TIE_GAME = "\
It\'s a tie\
";

class Checkers : public rclcpp::Node {
	public:
		static constexpr int BOARD_POSITIONS = 64;
		static constexpr int X_SIZE = 8;
		static constexpr int Y_SIZE = 8;

		Checkers();
		~Checkers();

		char boardState[Y_SIZE][X_SIZE]; // Current visual state
		char committedMoves[Y_SIZE][X_SIZE]; // Memory of committed moves
		char lastDetectedBoard[Y_SIZE][X_SIZE]; // Store last detect board
		
		char currentTurn; // R is robot and B is human
		bool gameActive;
		bool waitingForHuman;

		std::map<std::string, std::array<std::array<double, 3>, 2>> positions;
	private:
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commandPublisher_;
		rclcpp::Subscription<darknet_emulator_msgs::msg::BoundingBoxes>::SharedPtr boundingBoxesSubscriber_;
		size_t count_;
	
		std::thread inputThread; // Create keyboard input thread for reset
		std::atomic<bool> stopThread{false};

		void handleKeyboardInput();
		std::tuple<int, int> findBestMove();
		void boundingBoxesCallback(const darknet_emulator_msgs::msg::BoundingBoxes data);
		bool checkWinner();
		void clearTerminal();
		void displayBoard();
};

} /* pincergames */
