#include <iostream>
#include <thread>
#include <chrono>

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
			pincergames::Board newBoard = pincergames::minimax(game.getBoard(), 4, pincergames::Color::BLACK, game);
			//newBoard.draw(); //DEBUG LINE
			game.aiMove(newBoard);
		
		} else if (game.turn == pincergames::Color::RED) {
			//position = choosePosition(game.getBoard(), pincergames::Color::RED);
			//game.select(position);
			//game.select(chooseMove(game.getBoard(), position));
			
			pincergames::Board newBoard = pincergames::minimax(game.getBoard(), 4, pincergames::Color::RED, game);
			//newBoard.draw(); //DEBUG LINE
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
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	
	return 0;
}

/*
std::tuple<int, int> choosePosition(pincergames::Board board, pincergames::Color color) {
	
}

std::tuple<int, int> chooseMove(pincergames::Board board, std::tuple<int, int> position) {

}
*/
