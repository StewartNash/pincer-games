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

Board::Board() {
	redLeft = 12;
	blackLeft = 12;
	redKings = 0;
	blackKings = 0;
	createBoard();
}



double Board::evaluate() {
	double output;
	
	output = blackLeft - redLeft;
	output += 0.5 * blackKings - 0.5 * redKings;
	
	return output;
}

std::vector<Piece> Board::getAllPieces(Color color) {
	std::vector<Piece> pieces;
	
	return pieces;
}

void Board::move(Piece piece, int row, int column) {

}

Piece Board::getPiece(int row, int column) {
	return _board[row][column];
}

void Board::createBoard() {
	for (int row = 0; row < Checkers::Y_SIZE; ++row) {
		for (int column = 0; column < Checkers::X_SIZE; ++column) {
			// Only place pieces on dark squares
			if ((row + column) % 2 == 1) {
				if (row < 3) {	// Red pieces in top 3 rows
					board[row].push_back({row, column, Color::RED, false});
					_board[row][column] = {row, column, Color::RED, false};
				} else if (row > 4) {  // Black pieces in bottom 3 rows
					board[row].push_back({row, column, Color::BLACK, false});
					_board[row][column] = {row, column, Color::BLACK, false};
				} else {// Middle rows remain empty; no pieces added.
					_board[row][column] = _board[row][column] = {row, column, Color::NONE, false};
				}
			} else {
				_board[row][column] = _board[row][column] = {row, column, Color::NONE, false};
			}
		}
	}
}

void Board::remove(std::vector<Piece> pieces) {

}

Color Board::winner() {
	if (redLeft <= 0) {
		return Color::BLACK;
	} else if (blackLeft <= 0) {
		return Color::RED;
	} else {
		return Color::NONE;
	}
}

std::vector<Moves> Board::getValidMoves(Piece piece) {
	std::vector<Moves> moves;
	
	return moves;
}

std::vector<Moves> Board::traverseLeft(int start, int stop, int step, Color color, int left, Moves skipped) {
	std::vector<Moves> moves;
	
	return moves;
}

std::vector<Moves> Board::traverseRight(int start, int stop, int step, Color color, int right, Moves skipped) {
	std::vector<Moves> moves;
	
	return moves;

}

