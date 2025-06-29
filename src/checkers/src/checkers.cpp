#include "checkers/checkers.hpp"

#include <algorithm>
#include <limits>

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

void Piece::makeKing() {
	isKing = true;
}

void Piece::move(int row_, int column_) {
	row = row_;
	column = column_;
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
	
	for (int row = 0; row < Checkers::Y_SIZE; row++) {
		for (Piece& piece : _board[row]) {
			if (piece.color == color) {
				pieces.push_back(piece);
			}
		}
	}
	
	return pieces;
}

void Board::move(Piece piece, int row, int column) {
	Piece temporary = board[piece.row][piece.column];
	board[piece.row][piece.column] = board[row][column];
	board[row][column] = temporary;
	piece.move(row, column);
	
	if (row == Checkers::Y_SIZE - 1 || row == 0) {
		piece.makeKing();
		if (piece.color == Color::BLACK) {
			blackKings += 1;
		} else {
			redKings += 1;
		}
	}
	copyBoard();	
}

Piece Board::getPiece(int row, int column) {
	return board[row][column];
}

void Board::createBoard() {
	for (int row = 0; row < Checkers::Y_SIZE; ++row) {
		for (int column = 0; column < Checkers::X_SIZE; ++column) {
			// Only place pieces on dark squares
			if ((row + column) % 2 == 1) {
				if (row < 3) {	// Red pieces in top 3 rows
					_board[row].push_back({row, column, Color::RED, false});
					board[row][column] = {row, column, Color::RED, false};
				} else if (row > 4) {  // Black pieces in bottom 3 rows
					_board[row].push_back({row, column, Color::BLACK, false});
					board[row][column] = {row, column, Color::BLACK, false};
				} else {// Middle rows remain empty; no pieces added.
					board[row][column] = board[row][column] = {row, column, Color::NONE, false};
				}
			} else {
				board[row][column] = board[row][column] = {row, column, Color::NONE, false};
			}
		}
	}
}

void Board::draw() {
	Color temporary;
	
	// Display the current game board in the terminal
	//clearTerminal();
	std::cout << "\n╔═══╦═══╦═══╦═══╦═══╦═══╦═══╦═══╗" << std::endl;
	for (int i = 0; i  < Checkers::Y_SIZE; i++) {
		std::cout << "║ ";
		for (int j = 0; j < Checkers::X_SIZE; j++) {
			//temporary = committedMoves[i][j];
			temporary = board[i][j].color;
			if (temporary == Color::NONE) {
				std::cout << ' ';
			} else if (temporary == Color::RED {
				std::cout << 'R';
			} else {
				std::cout << 'B';
			}
			std::cout << " ║ ";
		}
		//std::cout << " ║";
		std::cout << std::endl;
		if (i < Checkers::Y_SIZE - 1) {
			std::cout << "╠═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╣" << std::endl;
		}		
	}
	std::cout << "╚═══╩═══╩═══╩═══╩═══╩═══╩═══╩═══╝" << std::endl;
	
	/*
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
	*/
}

void Board::remove(std::vector<Piece> pieces) {
	for (Piece& piece : pieces) {
		board[piece.row][piece.column] = Piece();
		if (piece.color == Color::RED) {
			redLeft -= 1;
		} else if (piece.color == Color::BLACK) {
			blackLeft -= 1;
		}
	}	
	copyBoard();
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

Moves Board::getValidMoves(Piece piece) {
	Moves moves;
	int left, right, row;
	Moves temporary;
	
	left = piece.column - 1;
	right = piece.column + 1;
	row = piece.row;
	
	if (piece.color == Color::RED || piece.isKing) {
		temporary = traverseLeft(row - 1, std::max(row - 3, -1), -1, piece.color, left);
		for (const auto& pair : temporary) {
			temporary[pair.first] = pair.second;
		}
		temporary = traverseRight(row - 1, std::max(row - 3, -1), -1, piece.color, right);
		for (const auto& pair : temporary) {
			temporary[pair.first] = pair.second;
		}		
	}
	if (piece.color == Color::BLACK || piece.isKing) {
		temporary = traverseLeft(row + 1, std::min(row + 3, Checkers::Y_SIZE), 1, piece.color, left);
		for (const auto& pair : temporary) {
			temporary[pair.first] = pair.second;
		}
		temporary = traverseRight(row + 1, std::min(row + 3, Checkers::Y_SIZE), 1, piece.color, right);
		for (const auto& pair : temporary) {
			temporary[pair.first] = pair.second;
		}
	}
	
	return moves;
}

Moves Board::traverseLeft(int start, int stop, int step, Color color, int left, std::vector<Piece> skipped) {
	Moves moves;
	std::vector<Piece> last;
	Moves temporary;
	
	for (int r = start; r < stop; r += step) {
		if (left < 0) {
			break;
		}
		Piece current = board[r][left];
		if (current.color == Color::NONE) {
			if (!skipped.empty() && last.empty()) {
				break;
			} else if (!skipped.empty()) {
				auto key = std::make_tuple(r, left);
				std::vector<Piece> combined = last;
				combined.insert(combined.end(), skipped.begin(), skipped.end());
				moves[key] = combined;
			} else {
				auto key = std::make_tuple(r, left);
				moves[key] = last;
			}
			if (!last.empty()) {
				int row;
				if (step == -1) {
					row = std::max(r - 3, 0);
				} else {
					row = std::min(r + 3, Checkers::Y_SIZE);
				}
				temporary = traverseLeft(r + step, row, step, color, left - 1, last);
				for (const auto& pair : temporary) {
					temporary[pair.first] = pair.second;
				}
				temporary = traverseRight(r + step, row, step, color, left + 1, last);
				for (const auto& pair : temporary) {
					temporary[pair.first] = pair.second;
				}
			}
			break;
		} else if (current.color == color) {
			break;
		} else {
			last.assign(1, current);
		}		
		left -= 1;
	}
	
	return moves;
}

Moves Board::traverseRight(int start, int stop, int step, Color color, int right, std::vector<Piece> skipped) {
	Moves moves;
	std::vector<Piece> last;
	Moves temporary;
	
	for (int r = start; r < stop; r += step) {
		if (right >= Checkers::X_SIZE) {
			break;
		}
		Piece current = board[r][right];
		if (current.color == Color::NONE) {
			if (!skipped.empty() && last.empty()) {
				break;
			} else if (!skipped.empty()) {
				auto key = std::make_tuple(r, right);
				std::vector<Piece> combined = last;
				combined.insert(combined.end(), skipped.begin(), skipped.end());
				moves[key] = combined;
			} else {
				auto key = std::make_tuple(r, right);
				moves[key] = last;
			}
			if (!last.empty()) {
				int row;
				if (step == -1) {
					row = std::max(r - 3, 0);
				} else {
					row = std::min(r + 3, Checkers::Y_SIZE);
				}
				temporary = traverseLeft(r + step, row, step, color, right - 1, last);
				for (const auto& pair : temporary) {
					temporary[pair.first] = pair.second;
				}
				temporary = traverseRight(r + step, row, step, color, right + 1, last);
				for (const auto& pair : temporary) {
					temporary[pair.first] = pair.second;
				}
			}
			break;
		} else if (current.color == color) {
			break;
		} else {
			last.assign(1, current);
		}		
		right += 1;
	}
	
	return moves;

}

void Board::copyBoard() {
	for (int i = 0; i < Checkers::Y_SIZE; i++) {
		std::vector<Piece> temporary;
		for (int j = 0; j < Checkers::X_SIZE; j++) {
				if (board[i][j].color != Color::NONE) {
					temporary.push_back(board[i][j]);
				}
		}
		_board[i] = temporary;
	}	
}

Game::Game() {
	init();
}

void Game::update() {
	board.draw();
}

Color Game::winner() {
	return board.winner();
}

void Game::reset() {
	init();
}

bool Game::select(int row, int column) {
	bool result;
	Piece piece;
	
	if (selected.color != Color::NONE) {
		result = move(row, column);
		if (!result) {
			selected = Piece();
			select(row, column);
		}
	}
	
	piece = board.getPiece(row, column);
	if (piece.color == turn) {
		selected = piece;
		validMoves = board.getValidMoves(piece);
		
		return true;
	}
	
	return true;
}

bool Game::select(std::tuple<int, int> position) {
	int row, column;
	
	row = std::get<0>(position);
	column = std::get<1>(position);
	
	return select(row, column);
}

void Game::changeTurn() {
	validMoves = Moves();
	if (turn == Color::RED) {
		turn = Color::BLACK;
	} else {
		turn = Color::RED;
	}
}

Board Game::getBoard() {
	return board;
}

void Game::aiMove(Board board_) {
	makeMove(board_);
}

void Game::humanMove(Board board_) {
	makeMove(board_);
}

void Game::makeMove(Board board_) {
	board = board_;
	changeTurn();
}

void Game::init() {
	board = Board();
	turn = Color::RED;
}

bool Game::move(int row, int column) {
	Piece piece;
	
	piece = board.getPiece(row, column);
	auto key = std::make_tuple(row, column);
	if (selected.color != Color::NONE && piece.color == Color::NONE && validMoves.find(key) != validMoves.end()) {
		board.move(selected, row, column);
		std::vector<Piece> skipped = validMoves[key];
		if (!skipped.empty()) {
			board.remove(skipped);
		}
		changeTurn();
	} else {
		return false;
	}
	
	return true;
}

Board minimax(Board position, int depth, Color maxPlayer, Game game) {
	return minimaxBestMove(position, depth, maxPlayer, game);
}


Board minimaxBestMove(Board position, int depth, Color maxPlayer, Game game) {
	double maxEval, minEval;
	double evaluation;
	Board bestMove;
	
	if (depth == 0 || position.winner().color != Color::NONE) {
		//return position.evaluate();
		return position;
	}
	
	if (maxPlayer == Color::BLACK) {
		maxEval = -std::numeric_limits<double>::max();
		for (auto move : getAllMoves(position, Color::BLACK, game) {
			evaluation = minimaxMinEval(move, depth - 1, Color::BLACK, game);
			maxEval = std::max(maxEval, evaluation);
			if (maxEval == evaluation) {
				bestMove = move;
			}
		}
		//return maxEval;
		return bestMove;
	} else if (maxPlayer == Color::RED) {
		minEval = std::numeric_limits<double>::max();
		for (auto move : getAllMoves(position, Color::RED, game) {
			evaluation = minimaxMinEval(move, depth - 1, true, game);
			minEval = std::min(minEval, evaluation);
			if (minEval == evaluation) {
				bestMove = move;
			}
		}
		//return minEval;
		return bestMove;
	} else {
		std::cout << "ERROR: minimaxBestMove(...), no player selected." << std::endl;
		//return position.evaluate();
		return position;
	}
}

double minimaxMinEval(Board position, int depth, Color maxPlayer, Game game) {
	double maxEval, minEval;
	double evaluation;
	Board bestMove;
	
	if (depth == 0 || position.winner().color != Color::NONE) {
		return position.evaluate();
		//return position;
	}
	
	if (maxPlayer == Color::BLACK) {
		maxEval = -std::numeric_limits<double>::max();
		for (auto move : getAllMoves(position, Color::BLACK, game) {
			evaluation = minimaxMinEval(move, depth - 1, Color::BLACK, game);
			maxEval = std::max(maxEval, evaluation);
			if (maxEval == evaluation) {
				bestMove = move;
			}
		}
		return maxEval;
		//return bestMove;
	} else if (maxPlayer == Color::RED) {
		minEval = std::numeric_limits<double>::max();
		for (auto move : getAllMoves(position, Color::RED, game) {
			evaluation = minimaxMinEval(move, depth - 1, true, game);
			minEval = std::min(minEval, evaluation);
			if (minEval == evaluation) {
				bestMove = move;
			}
		}
		return minEval;
		//return bestMove;
	} else {
		std::cout << "ERROR: minimaxMinEval(...), no player selected." << std::endl;
		return position.evaluate();
		//return position;
	}
}

Board simulateMove(Piece piece, std::tuple<int, int> move, Board board, std::vector<Piece> skip) {
	//int row, column;
	
	board.move(piece, std::get<0>(move), std::get<1>(move));
	if (!skip.empty()) {
		board.remove(skip)
	}
	
	return board;
}

std::vector<Board> getAllMoves(Board board, Color color, Game game) {
	std::vector<Board> moves;
	Moves validMoves;
	
	for (Piece piece : board.getAllPieces(color)) {
		validMoves = board.getValidMoves(piece);
		for (const auto& pair : validMoves) {
			std::tuple<int, int> move = pair.first;
			std::vector<Piece> skip = pair.second;
			Board tempBoard = board;
			Piece tempPiece = tempBoard.getPiece(piece.row, piece.column);
			Board newBoard = simulateMove(tempPiece, move, tempBoard, game, skip);
			moves.append.(newBoard);	
		}
	}
	
	return moves;
}
