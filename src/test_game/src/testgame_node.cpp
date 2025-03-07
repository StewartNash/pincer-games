#include "test_game/testgame.hpp"

using namespace pincergames;

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TestGame>());
	rclcpp::shutdown();
	return 0;
}
