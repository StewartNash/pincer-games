#pragma once

#include <atomic>
#include <map>
#include <array>
#include <cstring>
#include <tuple>
#include <vector>

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

enum Color{ NONE, RED, BLACK };

class Piece {
	public:
		Piece() : row(0), column(0), color(Color::NONE), isKing(false) {};
		Piece(int row_, int column_, Color color_, bool isKing_) : row(row_), column(column_), color(color_), isKing(isKing_) {};
		int row, column;
		Color color;
		bool isKing;
		void makeKing();
		void move(int row_, int column_);
};

typedef std::map<std::tuple<int, int>, std::vector<Piece>> Moves;

class Board {
	public:
		Board();
		//TODO: Choose one representation, either 'board' or '_board'
		std::vector<Piece> _board[Checkers::Y_SIZE];
		Piece board[Checkers::Y_SIZE][Checkers::X_SIZE];
		int redLeft, blackLeft;
		int redKings, blackKings;

		double evaluate();
		std::vector<Piece> getAllPieces(Color color);
		void move(Piece piece, int row, int column);
		Piece getPiece(int row, int column);
		void createBoard();
		void remove(std::vector<Piece> pieces);
		Color winner();
		Moves getValidMoves(Piece piece);
	
	private:
		Moves traverseLeft(int start, int stop, int step, Color color, int left, std::vector<Piece> skipped = std::vector<Piece>());
		Moves traverseRight(int start, int stop, int step, Color color, int right, std::vector<Piece> skipped = std::vector<Piece>());
		void copyBoard();
};

class Game {
	public:
		Game();
		void update();
		Color winner();
		void reset();
		bool select(int row, int column);
		void changeTurn();
		Board getBoard();
		void aiMove(Board board);

	private:
		void init();
		bool move(int row, int column);
};

int minimax();

} /* pincergames */
