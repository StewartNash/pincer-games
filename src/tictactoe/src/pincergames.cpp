using namespace pincergames

TicTacToe::TicTacToe(void) : rclcpp::Node("tic_tac_toe_robot"), count_(0) {
	commandPublisher = this->create_publisher<std_msgs::msg::String>("ui_command", 10);
}
