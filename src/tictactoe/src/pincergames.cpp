#include "pincergames.hpp"

using namespace pincergames;
//using namespace std::chrono_literals

TicTacToe::TicTacToe() : rclcpp::Node("tic_tac_toe_robot"), count_(0) {
	std::fill(std::begin(boardState), std::end(boardState), '\0');
	std::fill(std::begin(committedMoves), std::end(committedMoves), '\0');
	std::fill(std::begin(lastDetectedBoard), std::end(lastDetectedBoard), '\0');

	currentTurn = 'X';
	currentFigure = 1;
	waitingForHuman = false;
	gameActive = true;

	inputThread = std::thread(&TicTacToe::handleKeyboardInput, this);

	positions["1AP"] = {{{{-1.300, 1.049, -0.818}}, {{0.000, 0.110, -0.267}}}};
	positions["1"] = {{{{-1.310, 1.197, -0.764}}, {{0.000, 0.312, -0.267}}}};
	positions["2AP"] = {{{{-1.101, 1.049, -0.766}}, {{0.000, 0.075, -0.616}}}};
	positions["2"] = {{{{-1.096, 1.183, -0.716}}, {{0.000, 0.232, -0.511}}}};
	positions["3AP"] = {{{{-0.906, 1.049, -0.668}}, {{0.000, 0.075, -0.826}}}};
	positions["3"] = {{{{-0.916, 1.171, -0.607}}, {{0.000, 0.075, -0.721}}}};
	positions["4AP"] = {{{{-0.766, 1.049, -0.529}}, {{0.000, -0.099, -1.035}}}};
	positions["4"] = {{{{-0.766, 1.173, -0.431}}, {{0.000, -0.099, -0.930}}}};
	positions["5AP"] = {{{{-0.661, 1.066, -0.396}}, {{0.000, -0.099, -1.209}}}};
	positions["5"] = {{{{-0.668, 1.199, -0.274}}, {{0.000, -0.274, -1.105}}}};
	positions["T1AP"] = {{{{-0.888, 1.129, -0.103}}, {{0.000, -0.518, -0.930}}}};
	positions["T1"] = {{{{-0.888, 1.247, -0.016}}, {{0.000, -0.518, -0.756}}}};
	positions["T2AP"] = {{{{-1.004, 1.246, 0.265}}, {{0.000, -0.780, -0.756}}}};
	positions["T2"] = {{{{-1.004, 1.373, 0.363}}, {{0.000, -0.780, -0.651}}}};
	positions["T3AP"] = {{{{-1.080, 1.456, 0.754}}, {{0.000, -1.042, -0.581}}}};
	positions["T3"] = {{{{-1.080, 1.595, 0.925}}, {{0.000, -1.077, -0.581}}}};
	positions["T4AP"] = {{{{-1.087, 1.045, -0.335}}, {{0.000, -0.274, -0.581}}}};
	positions["T4"] = {{{{-1.087, 1.193, -0.234}}, {{0.000, -0.379, -0.581}}}};
	positions["T5AP"] = {{{{-1.164, 1.167, 0.066}}, {{0.000, -0.641, -0.581}}}};
	positions["T5"] = {{{{-1.164, 1.289, 0.169}}, {{0.000, -0.641, -0.477}}}};
	positions["T6AP"] = {{{{-1.230, 1.359, 0.534}}, {{0.000, -0.955, -0.477}}}};
	positions["T6"] = {{{{-1.221, 1.471, 0.639}}, {{0.000, -0.955, -0.372}}}};
	positions["T7AP"] = {{{{-1.314, 1.045, -0.431}}, {{0.000, -0.274, -0.232}}}};
	positions["T7"] = {{{{-1.314, 1.176, -0.339}}, {{0.000, -0.274, -0.232}}}};
	positions["T8AP"] = {{{{-1.370, 1.145, -0.042}}, {{0.000, -0.606, -0.302}}}};
	positions["T8"] = {{{{-1.370, 1.258, 0.045}}, {{0.000, -0.606, -0.197}}}};
	positions["T9AP"] = {{{{-1.387, 1.284, 0.412}}, {{0.000, -0.955, -0.302}}}};
	positions["T9"] = {{{{-1.387, 1.429, 0.520}}, {{0.000, -0.955, -0.197}}}};
	positions["TOP_CENTER"] = {{{{-0.136, 1.018, 1.639}}, {{0.000, -0.640, -0.581}}}};

	resetGame();

	clearTerminal();
	displayBoard();
	std::cout << "\nWaiting for moves... (X: Robot, O: Human)" << std::endl;
	std::cout << "\nPress 'r' and Enter to restart the game at any time" << std::endl;

	commandPublisher_ = this->create_publisher<std_msgs::msg::String>("ui_command", 10);
	////timer_ = this->create_wall_timer(500ms, std::bind(&TicTacToe::timer_callback, this));
	//timer_ = this->create_wall_timer(std::literals::chrono_literals::operator""ms(500), std::bind(&TicTacToe::timer_callback, this));
	//subscription_ = this->create_subscription<std_msgs::msg::String>("/darknet_ros/bounding_boxes", darknet_emulator::msg::BoundingBoxes, boundingBoxesCallback, 1);
	boundingBoxesSubscriber_ = this->create_subscription<darknet_emulator::msg::BoundingBoxes>("darknet_emulator/bounding_boxes", 1,  std::bind(&TicTacToe::boundingBoxesCallback, this, std::placeholders::_1));
}

TicTacToe::~TicTacToe() {
	stopThread.store(true);
	if (inputThread.joinable()) {
		inputThread.join();
	}

	commandPublisher_.reset();
	//timer_.reset();
}
/*
void TicTacToe::timer_callback() {
	auto message = std_msgs::msg::String();
	message.data = "TicTacToe " + std::to_string(count_++);
	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
	publisher_->publish(message);
}
*/
void TicTacToe::handleKeyboardInput() {
	char userInput;
	while (!stopThread.load()) {
		std::cin >> userInput;
		if (userInput == 'r') {
			resetGame();
			//break;
		}
	}
}

void TicTacToe::resetGame() {
	std::fill(std::begin(boardState), std::end(boardState), '\0');
	std::fill(std::begin(committedMoves), std::end(committedMoves), '\0');
	std::fill(std::begin(lastDetectedBoard), std::end(lastDetectedBoard), '\0');

	currentFigure = 1;
	waitingForHuman = true;
	gameActive = true;
	
	clearTerminal();
	displayBoard();
	std::cout << "\nGame Reset! Waiting for human move..." << std::endl;

	clearTerminal();
	displayBoard();
	std::cout << "\nWaiting for moves... (X: Robot, O: Human)" << std::endl;
}

int TicTacToe::findBestMove() {
	// Check if we can win in the next move
	for (int i = 0; i < BOARD_POSITIONS; i++) {
		if (boardState[i] == '\0') {
			boardState[i] = 'X';
			if (checkWinner()) {
				boardState[i] = '\0';
				return i;
			}
			boardState[i] = '\0';
		}
	}
	
	// Check if opponent can win in their next move and block them
	for (int i = 0; i < BOARD_POSITIONS; i++) {
		if (boardState[i] == '\0') {
			boardState[i] = 'O';
			if checkWinner() {
				boardState[i] = '\0';
				return i;
			}
			boardState[i] = '\0';
		}
	}
	
	// Take center if available
	if (boardState[CENTER_POSITION] == '\0') {
		return CENTER_POSITION;
	}	
	
	return -1;
}

void TicTacToe::boundingBoxesCallback(const darknet_emulator::msg::BoundingBoxes data) const {
	char currentBoard[BOARD_POSITIONS];
	
	// Process all detected symbols and update the board state
	if (!gameActive || !waitingForHuman) {
		return;
	}

	std::strcpy(currentBoard, committedMoves);

	// Process all detected boxes
	for (darknet_emulator::msg::BoundingBox obj : data.bounding_boxes) {
	
	}
	
	// Check if new valid O move has been made
	if (!std::strcmp(currentBoard, lastDetectedBoard)) {

	}
	return;
}

void TicTacToe::clearTerminal() {
	std::cout << "\033[2J\033[1;1H";
}

bool TicTacToe::checkWinner() {
	// Check for a winner in any direction
	
	return false;
}

void TicTacToe::displayBoard() {
	int row[BOARD_SIZE];
	int startPosition;
	int endPosition;
	int sliceLength;
	
	// Display the current game board in the terminal
	clearTerminal();
	std::cout << "\n╔═══╦═══╦═══╗" << std::endl;
	for (int i = 0; i  < BOARD_SIZE; i++) {
		startPosition = i * BOARD_SIZE;
		endPosition = (i + 1) * BOARD_SIZE;
		sliceLength = endPosition - startPosition;
		// Assign slice
		for (int j = 0; j < sliceLength; j++) {
			char temporary;
			temporary = committedMoves[startPosition + j];
			if (temporary == '\0') {
				row[j] = ' ';
			} else {
				row[j] = temporary;
			}
		}
		std::cout << "║ ";
		std::cout << row[0];
		std::cout << " ║ ";
		std::cout << row[1];
		std::cout << " ║ ";
		std::cout << row[2];
		std::cout << " ║";
		std::cout << std::endl;
		if (i < 2) {
			std::cout << "╠═══╬═══╬═══╣" << std::endl;
		}		
	}
	std::cout << "╚═══╩═══╩═══╝" << std::endl;
	
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
