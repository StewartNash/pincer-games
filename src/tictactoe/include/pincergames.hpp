#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pincergames {

const std::string ROBOT_WINS = "\
  ____       *           *              _           \n\
 |  * \\ *** | |**   ___ | |_  __      *(*)_ **  **_ \n\
 | |_) / * \\| '* \\ / * \\| *_| \\ \\ /\\ / / | '_ \\/ __|\n\
 |  * < (*) | |_) | (_) | |_   \\ V  V /| | | | \\__ \\\n\
 |_| \\_\\___/|_.__/ \\___/ \\__|   \\_/\\_/ |_|_| |_|___/\
";

const std::string HUMAN_WINS = "\
  *   *                                        _           \n\
 | | | |_   * ***_**   ** ***** ***   __      *(*)_ **  **_ \n\
 | |_| | | | | '_ `  \\ / ` | '_ \\  \\ \\ /\\ / / | '_ \\/ __|\n\
 |  *  | |*| | | | | | | (_| | | | |  \\ V  V /| | | | \\__ \\\n\
 |_| |_|\\__,_|_| |_| |_|\\__,_|_| |_|   \\_/\\_/ |_|_| |_|___/\
";

const std::string TIE_GAME = "\
  ___ *   *               *   *      \n\
 |_ *| |*( )___    __ *  | |*(_) ___ \n\
  | || **|// **|  / *` | | *_| |/ _ \\\n\
  | || |_  \\__ \\ | (_| | | |_| |  __/\n\
 |___|\\__| |___/  \\__,_|  \\__|_|\\___|\
";

class TicTacToe : public rclcpp::Node {
	public:
		TicTacToe();
		
		char boardState[9];
		char committedMoves[9];
		char currentTurn;
		short currentFigure;
		char lastDetectedBoard[9];
		bool waitingForHuman;
		bool gameActive;
		
		
}

}
