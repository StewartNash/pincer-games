#include "pincergames.hpp"

using namespace pincergames;
//using namespace std::chrono_literals

TicTacToe::TicTacToe() : rclcpp::Node("tic_tac_toe_robot"), count_(0) {
	publisher_ = this->create_publisher<std_msgs::msg::String>("ui_command", 10);
	//timer_ = this->create_wall_timer(500ms, std::bind(&TicTacToe::timer_callback, this));
	timer_ = this->create_wall_timer(std::literals::chrono_literals::operator""ms(500), std::bind(&TicTacToe::timer_callback, this));
}

void TicTacToe::timer_callback() {
	auto message = std_msgs::msg::String();
	message.data = "TicTacToe " + std::to_string(count_++);
	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
	publisher_->publish(message);
}
