#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pincergames {

const std::string ROBOT_WINS = "";

const std::string HUMAN_WINS = "";

const std::string TIE_GAME = "";

class TicTacToe : public rclcpp::Node {
	public:
		TicTacToe();
}

}
