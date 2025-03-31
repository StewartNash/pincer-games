#include "checkers/checkers.hpp"

using namespace pincergames;

Checkers::Checkers() : rclcpp::Node("checkers_robot"), count_(0) {
	std::memset(boardState, '\0', sizeof(boardState));
	
	currentTurn = 'R';
	waitingForHuman = false;
	gameActive = true;
	
	displayBoard();
	
	commandPublisher_ = this->create_publisher<std_msgs::msg::String>("ui_command", 10);
	boundingBoxesSubscriber_ = this->create_subscription<darknet_emulator_msgs::msg::BoundingBoxes>("bounding_boxes", 1, std::bind(&Checkers::boundingBoxesCallback, this, std::placeholders::_1));
}

Checkers::~Checkers() {

}

void Checkers::handleKeyboardInput() {

}

void Checkers::boundingBoxesCallback(const darknet_emulator_msgs::msg::BoundingBoxes data) {

}

std::tuple<int, int> Checkers::findBestMove() {
	std::tuple<int, int> output;
	
	output = std::make_tuple(0, 0);
	
	return output;
}

bool Checkers::checkWinner() {
	return false;
}

void Checkers::clearTerminal() {

}

void Checkers::displayBoard() {
	char temporary;
	
	// Display the current game board in the terminal
	clearTerminal();
	std::cout << "\n╔═══╦═══╦═══╦═══╦═══╦═══╦═══╦═══╗" << std::endl;
	for (int i = 0; i  < Y_SIZE; i++) {
		std::cout << "║ ";
		for (int j = 0; j < X_SIZE; j++) {
			//temporary = committedMoves[i][j];
			temporary = boardState[i][j];
			if (temporary == '\0') {
				std::cout << ' ';
			} else {
				std::cout << temporary;
			}
			std::cout << " ║ ";
		}
		//std::cout << " ║";
		std::cout << std::endl;
		if (i < Y_SIZE - 1) {
			std::cout << "╠═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╣" << std::endl;
		}		
	}
	std::cout << "╚═══╩═══╩═══╩═══╩═══╩═══╩═══╩═══╝" << std::endl;
	
	// Print current state
	if (!gameActive) {
		if (checkWinner()) {
			if (currentTurn == 'X') {
				std::cout << HUMAN_WINS;
			} else {
				std::cout << ROBOT_WINS;
			}
		} else {
			std::cout << TIE_GAME;
		}
		std::cout << "\nPress 'r' and Enter to restart" << std::endl;
	} else if (waitingForHuman) {
		std::cout << "\nWaiting for human move..." << std::endl;
	} else {
		std::cout << "\nRobot is thinking..." << std::endl;
	}
}
