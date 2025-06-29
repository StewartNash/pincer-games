#include "checkers.hpp"

//std::tuple<int, int> choosePosition(pincergames::Board board, pincergames::Color color);
//std::tuple<int, int> chooseMove(pincergames::Board board, std::tuple<int, int> position);

int main(int argc, char* argv[]) {
	pincergames::Game game;
	bool run;
	std::tuple<int, int> position;
	
	run = true;
	game = pincergames::Game();
	while (run) {
		if (game.turn == pincergames::Color::BLACK) {
			Board newBoard = pincergames::minimax(game.getBoard(), 4, pincergames::Color::BLACK, game);
			game.aiMove(newBoard);
		
		} else if (game.turn == pincergames::Color::RED) {
			//position = choosePosition(game.getBoard(), pincergames::Color::RED);
			//game.select(position);
			//game.select(chooseMove(game.getBoard(), position));		
			Board newBoard = pincergames::minimax(game.getBoard(), 4, pincergames::Color::RED, game);
			game.humanMove(newBoard);
		}
		
		if (game.winner() != pincergames::Color::NONE) {
			if (game.winner() == pincergames::Color::RED) {
				std::cout << pincergames::RED_WINS << std::endl;
			} else {
				std::cout << pincergames::BLACK_WINS << std::endl;
			}
			run = false;		
		}
		
		game.update();
	}
	
	return 0;
}

/*
std::tuple<int, int> choosePosition(pincergames::Board board, pincergames::Color color) {
	
}

std::tuple<int, int> chooseMove(pincergames::Board board, std::tuple<int, int> position) {

}
*/
