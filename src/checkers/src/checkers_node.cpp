#include "checkers/checkers.hpp"

using namespace pincergames;

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Checkers>());
	rclcpp::shutdown();
	return 0;
}
