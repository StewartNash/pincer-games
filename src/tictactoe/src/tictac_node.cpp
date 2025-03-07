#include "tictactoe/tictac.hpp"

using namespace pincergames;

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TicTacToe>());
	rclcpp::shutdown();
	return 0;
}
