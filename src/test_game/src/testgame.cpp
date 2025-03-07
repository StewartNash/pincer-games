#include "test_game/testgame.hpp"

using namespace pincergames;

TestGame::TestGame() : rclcpp::Node("tesst_game_robot"), count_(0) {
	publisher_ = this->create_publisher<std_msgs::msg::String>("ui_command", 10);
	subscriber_ = this->create_subscription<darknet_emulator_msgs::msg::BoundingBoxes>("darknet_emulator_msgs/bounding_boxes", 1, std::bind(&TestGame::callback, this, std::placeholders::_1));
}

TestGame::~TestGame() {

}

void TestGame::callback(const darknet_emulator_msgs::msg::BoundingBoxes data) {

}
