#include "tictactoe/tictac.hpp"

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
	boundingBoxesSubscriber_ = this->create_subscription<darknet_emulator_msgs::msg::BoundingBoxes>("darknet_emulator_msgs/bounding_boxes", 1,  std::bind(&TicTacToe::boundingBoxesCallback, this, std::placeholders::_1));
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
	const int NUMBER_OF_CORNERS = 4;
	const int NUMBER_OF_EDGES = 4;
	int corners[NUMBER_OF_CORNERS] = {0, 2, 6, 8};
	int edges[NUMBER_OF_EDGES] = {1, 3, 5, 7};
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
			if (checkWinner()) {
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

	// Take corners if available
	for (int i = 0; i < NUMBER_OF_CORNERS; i++) {
		if (boardState[corners[i]] == '\0') {
			//int availableCorner = corners[i];
			//return availableCorner;
			return corners[i];
		}
	}

	// Take any available edge
	for (int i = 0; i < NUMBER_OF_EDGES; i++) {
		if (boardState[edges[i]] == '\0') {
			//int availableEdge = edges[i];
			//return availableEdge;
			return edges[i];
		}
	}
	return -1;
}

void TicTacToe::boundingBoxesCallback(const darknet_emulator_msgs::msg::BoundingBoxes data) {
	char currentBoard[BOARD_POSITIONS];
	int moveIndex;
	std::vector<int> newOMoves;
	int bestMove;
	
	// Process all detected symbols and update the board state
	if (!gameActive || !waitingForHuman) {
		return;
	}

	std::strcpy(currentBoard, committedMoves);

	// Process all detected boxes
	for (darknet_emulator_msgs::msg::BoundingBox box : data.bounding_boxes) {
		moveIndex = mapBoundingBoxToGrid(box);
		if (moveIndex >=0) { // Only process moves within boundaries
			if (box.class_id == "oxx" && committedMoves[moveIndex] == '\0') {
				currentBoard[moveIndex] = 'O';
			} else if (box.class_id == "ixx") {
				// For X moves, only show them if they're committedMOves
				if (committedMoves[moveIndex] == 'X') {
					currentBoard[moveIndex] = 'X';
				}
			}
		}
	}
	
	// Check if new valid O move has been made
	if (!std::strcmp(currentBoard, lastDetectedBoard)) {
		for (int i = 0; i < BOARD_POSITIONS; i++) {
			if (currentBoard[i] == 'O' && committedMoves[i] == '\0') {
				newOMoves.push_back(i);
			}
		}
		if (newOMoves.size() == 1) { // Exactly one new O move
			moveIndex = newOMoves[0];
			committedMoves[moveIndex] = 'O'; // Commit the move
			std::strcpy(boardState, committedMoves);
			std::strcpy(lastDetectedBoard, boardState);
			waitingForHuman = false;
			
			clearTerminal();
			displayBoard();
			
			if (checkWinner()) {
				std::cout << "\nHuman wins!" << std::endl;
				//TODO: display HUMAN_WINS
				gameActive = false;
				return;
			}
			
			// Make robot's move
			bestMove = findBestMove();
			if (bestMove >= 0) {
				committedMoves[bestMove] = 'X'; // Commit the robot's move
				std::strcpy(boardState, committedMoves);
				pickAndPlace(bestMove);
				currentFigure += 1;
				
				clearTerminal();
				displayBoard();
			}
			
			if (checkWinner()) {
				std::cout << "\nRobot wins!" << std::endl;
				std::cout << ROBOT_WINS << std::endl;
				gameActive = false;
				return;
			} else { //TODO: Use lambda expression?
				bool isFull = true;
				for (int i = 0; i < BOARD_POSITIONS; i++) {
					if (committedMoves[i] == '\0') {
						isFull = false;
					}
				}
				if (isFull) {
					std::cout << "\nGameOver - It\'s a tie!" << std::endl;
					std::cout << TIE_GAME << std::endl;
					return;
			      	}
			}
			
			waitingForHuman = true;
			displayBoard();
		}
	}
	return;
}

std::array<double, 3> TicTacToe::applyJointOffsets(std::array<double, 3> coords) {
	// Modified to handle complete joint state
	// Returns the complete position tuple structure
	return coords; // Since we're using full joint states, we don't need to modify them
}

std::array<std::array<double, 3>, 2> TicTacToe::applyJointOffsets(std::array<std::array<double, 3>, 2> coords) {
	// Modified to handle complete joint state
	// Returns the complete position tuple structure
	return coords; // Since we're using full joint states, we don't need to modify them
}

void TicTacToe::clearTerminal() {
	std::cout << "\033[2J\033[1;1H";
}

int TicTacToe::mapBoundingBoxToGrid(darknet_emulator_msgs::msg::BoundingBox box) {
	double boxXCenter, boxYCenter;
	double leftBoundary, rightBoundary, topBoundary, bottomBoundary;
	double xPos, yPos;;
	int gridX, gridY;

	boxXCenter = (box.xmin + box.xmax) / 2.0;
	boxYCenter = (box.ymin + box.ymax) / 2.0;

	leftBoundary = 197.0;
	rightBoundary = 598.0;
	topBoundary = 25.0;
	bottomBoundary = 431.0;

	xPos = (boxXCenter - leftBoundary) / (rightBoundary - leftBoundary);
	yPos = (boxYCenter - topBoundary) / (bottomBoundary - topBoundary);

	gridX = static_cast<int>(xPos * 3.0);
	gridY = static_cast<int>(yPos * 3.0);

	if (0 <= gridX && gridX <= 3 && 0 <= gridY && gridY < 3) {
		return gridY * 3 + gridX;
	}
	return -1;
}
	
bool TicTacToe::checkWinner() {
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

		if (boardState[a] == boardState[b] &&
			boardState[b] == boardState[c] &&
			boardState[c] != '\0') {
			return true;
		}
	}
	
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

void TicTacToe::moveToPosition(std::string positionKey) {
	// Move to a specific position with approach position handling
	std::array<std::array<double, 3>, 2> apPosition;
	std::array<std::array<double, 3>, 2> targetPosition;

	if (positions.count(positionKey) > 0) { // Key exists
		std::cout << "\nMoving to position: " << positionKey << std::endl;

		// Move to approach position
		std::cout << "-> Approach position " << positionKey << "AP";
		std::cout << std::endl;
		apPosition = positions[positionKey + "AP"]; // get complete position tuple
		moveToCoordinates(apPosition);

		// Move to target position
		std::cout << "-> Target position " << positionKey << std::endl;
		targetPosition = positions[positionKey]; // Get complete position tuple
		moveToCoordinates(targetPosition);

		// Return to approach position
		std::cout << "-> Return to approach position " << positionKey << "AP";
		std::cout << std::endl;
		moveToCoordinates(apPosition);
	} else {
		//TODO: Throw error
		std::cout << "[void TicTacToe::moveToPosition(std::string positionKey)]";
		std::cout << " ERROR - Invalid position reference: " << positionKey;
		std::cout << std::endl;
	}
}

void TicTacToe::pickAndPlace(int targetPosition) {
	std::string figurePos, tablePos;
	auto temporary = std_msgs::msg::String();
	
	figurePos = std::to_string(currentFigure);
	tablePos = "T" + std::to_string(targetPosition + 1);
	
	updateDisplay("Starting pick and place sequence for figure " +
	  figurePos + " to position " + tablePos);
	
	// Pick sequence
	updateDisplay("Opening gripper");
	temporary.data = "open_gripper";
	commandPublisher_->publish(temporary);
	//xogcode.main(null, serialPort, "open");

	updateDisplay("Moving to approach position " + figurePos + "AP");
	moveToCoordinates(positions[figurePos + "AP"]);
	//xogcode.main(figurePos + "AP", serialPort);
	
	// Continue with all movements, updating display each time
	updateDisplay("Moving to pick position " + figurePos);
	moveToCoordinates(positions[figurePos]);
	//xogcode.main(figurePos, serialPort);
	
	updateDisplay("Closing gripper");
	temporary.data = "close_gripper";
	commandPublisher_->publish(temporary);
	//xogcode.main(null, serialPort, "close");
	
	updateDisplay("Returning to approach position " + figurePos + "AP");
	moveToCoordinates(positions[figurePos + "AP"]);
	//xogcode.main(figurePos + "AP", serialPort);
	
	updateDisplay("Moving to top position");
	//xogcode.main("top", serialPort);
	
	updateDisplay("Moving to approach position " + tablePos + "AP");
	moveToCoordinates(positions[tablePos]);
	//xogcode.main(tablePos, serialPort);
	
	updateDisplay("Opening gripper");
	temporary.data = "open_gripper";
	commandPublisher_->publish(temporary);
	//xogcode.main(null, serialPort, "open");
	
	updateDisplay("Returning to approach position " + tablePos + "AP");
	moveToCoordinates(positions[tablePos + "AP"]);
	//xogcode.main(tablePos + "AP", serialPort);
	
	updateDisplay("Moving to top position");
	//xogcode.main("top", serialPort);
	
	updateDisplay("Moving to park position");
	//xogcode.main("park", serialPort);
}

void TicTacToe::moveToCoordinates(double coords[3]) {
	double j1, j2, j3, j4, j5, j6;

	j1 = coords[0];
	j2 = coords[1];
	j3 = coords[2];
	j4 = 0.0;
	j5 = 0.0;
	j6 = 0.0;

	auto command = std_msgs::msg::String();
	//TODO: Set precision to 4
	command.data = "go_to_joint_state," + std::to_string(j1) + ",";
	command.data += std::to_string(j2) + "," + std::to_string(j3) + ",";
	command.data += std::to_string(j4) + "," + std::to_string(j5) + ",";
	command.data += std::to_string(j6);
	std::cout << "\n==================================================";
	std::cout << std::endl << "SENDING MOVEMENT COMMAND:" << std::endl;
	std::cout << "Raw coordinates: " << std::to_string(j1) << ",";
	std::cout << std::to_string(j2) << "," << std::to_string(j3);
	std::cout << std::endl;
	std::cout << "==================================================\n";
	
	commandPublisher_->publish(command);
	//TODO: Insert 0.5 second sleep
}

void TicTacToe::moveToCoordinates(double coords[3][2]) {
	double j1, j2, j3, j4, j5, j6;

	j1 = coords[0][0];
	j2 = coords[1][0];
	j3 = coords[2][0];
	j4 = coords[0][1];
	j5 = coords[1][1];
	j6 = coords[2][1];

	auto command = std_msgs::msg::String();
	//TODO: Set precision to 4
	command.data = "go_to_joint_state," + std::to_string(j1) + ",";
	command.data += std::to_string(j2) + "," + std::to_string(j3) + ",";
	command.data += std::to_string(j4) + "," + std::to_string(j5) + ",";
	command.data += std::to_string(j6);
	std::cout << "\n==================================================";
	std::cout << std::endl << "SENDING MOVEMENT COMMAND:" << std::endl;
	std::cout << "Raw coordinates: " << std::to_string(j1) << ",";
	std::cout << std::to_string(j2) << "," << std::to_string(j3);
	std::cout << " " << std::to_string(j4) << "," << std::to_string(j5);
	std::cout << "," << std::to_string(j6) << std::endl;
	std::cout << "==================================================\n";
	
	commandPublisher_->publish(command);
	//TODO: Insert 0.5 second sleep
}

void TicTacToe::moveToCoordinates(std::array<double, 3> coords) {
	double output[3];
	
	for (int i = 0; i < 3; i++) {
		  output[i] = coords[i];
	}
	moveToCoordinates(output);	
}

void TicTacToe::moveToCoordinates(std::array<std::array<double, 3>, 2> coords) {
	double output[3][2];
	
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
		  output[j][i] = coords[i][j];
		}
	}
	moveToCoordinates(output);	
}

std::string TicTacToe::updateGripperState(std::string action) {
	std::string output = "gripper_command,";
	
	output += (action == "open") ? "open" : "close";

	return output;
}

void TicTacToe::updateDisplay(std::string status) {
	clearTerminal();
	displayBoard();
	std::cout << "\nRobot action: " << status << std::endl;
}
