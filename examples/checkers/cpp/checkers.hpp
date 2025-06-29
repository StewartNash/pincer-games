#pragma once

#include <atomic>
#include <map>
#include <array>
#include <cstring>
#include <string>
#include <tuple>
#include <vector>

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

const std::string RED_WINS = "\
Red wins\
";

const std::string BLACK_WINS = "\
Black wins\
";


class Checkers {
	public:
		static constexpr int BOARD_POSITIONS = 64;
		static constexpr int X_SIZE = 8;
		static constexpr int Y_SIZE = 8;
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
		void draw();
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
		bool select(std::tuple<int, int> position);
		void changeTurn();
		Board getBoard();
		void aiMove(Board board_);
		void humanMove(Board board_);
		void makeMove(Board board_);

		Piece selected;
		Board board;
		Color turn;
		Moves validMoves;

	private:
		void init();
		bool move(int row, int column);
};

Board minimax(Board position, int depth, Color maxPlayer, Game game);
Board minimaxBestMove(Board position, int depth, Color maxPlayer, Game game);
double minimaxMinEval(Board position, int depth, Color maxPlayer, Game game);
Board simulateMove(Piece piece, std::tuple<int, int> move, Board board, std::vector<Piece> skip);
std::vector<Board> getAllMoves(Board board, Color color, Game game);
//void drawMoves(Game game, Board board, Piece piece);

} /* pincergames */
