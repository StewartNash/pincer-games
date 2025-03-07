#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from darknet_ros_msgs.msg import BoundingBoxes
import math
import threading
import os
import sys
sys.path.append('/home/zz/catkin_workspace/src/darknet_ros/darknet/scripts/xogcodes')  
import xogcode  # Import the xogcode module
from xogcode import find_serial_port


# ASCII Art Messages
ROBOT_WINS = """
  ____       *           *              _           
 |  * \ *** | |**   ___ | |_  __      *(*)_ **  **_ 
 | |_) / * \| '* \ / * \| *_| \ \ /\ / / | '_ \/ __|
 |  * < (*) | |_) | (_) | |_   \ V  V /| | | | \__ \\
 |_| \_\___/|_.__/ \___/ \__|   \_/\_/ |_|_| |_|___/
"""

HUMAN_WINS = """
  *   *                                        _           
 | | | |_   * ***_**   ** ***** ***   __      *(*)_ **  **_ 
 | |_| | | | | '_ `  \ / ` | '_ \  \ \ /\ / / | '_ \/ __|
 |  *  | |*| | | | | | | (_| | | | |  \ V  V /| | | | \__ \\
 |_| |_|\__,_|_| |_| |_|\__,_|_| |_|   \_/\_/ |_|_| |_|___/
"""

TIE_GAME = """
  ___ *   *               *   *      
 |_ *| |*( )___    __ *  | |*(_) ___ 
  | || **|// **|  / *` | | *_| |/ _ \\
  | || |_  \__ \ | (_| | | |_| |  __/
 |___|\__| |___/  \__,_|  \__|_|\___|
"""

class TicTacToe:
    def __init__(self):
        rospy.init_node('tic_tac_toe_robot', anonymous=True)
        self.serial_port = find_serial_port()

        # Publisher for robot commands
        self.command_publisher = rospy.Publisher('ui_command', String, queue_size=10)
        
        # Subscriber for YOLO bounding boxes
        self.bounding_boxes_subscriber = rospy.Subscriber(
            '/darknet_ros/bounding_boxes', 
            BoundingBoxes, 
            self.bounding_boxes_callback,
            queue_size=1
        )
        
        # Game state
        self.board_state = [""] * 9  # Current visual state
        self.committed_moves = [""] * 9  # Memory of committed moves
        self.current_turn = 'X'  # X is robot, O is human
        self.current_figure = 1  # Track which figure to use next (1-5)
        self.last_detected_board = [""] * 9  # Store last detected board state
        self.waiting_for_human = False
        self.game_active = True
        
        # Create a keyboard input thread for reset
        self.input_thread = threading.Thread(target=self.handle_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()


        # Define all positions (in radians) including approach positions
        self.positions = {
        "1AP": [(-1.300, 1.049, -0.818), (0.000, 0.110, -0.267)],
        "1": [(-1.310, 1.197, -0.764), (0.000, 0.312, -0.267)],
        
        "2AP": [(-1.101, 1.049, -0.766), (0.000, 0.075, -0.616)],
        "2": [(-1.096, 1.183, -0.716), (0.000, 0.232, -0.511)],
        
        "3AP": [(-0.906, 1.049, -0.668), (0.000, 0.075, -0.826)],
        "3": [(-0.916, 1.171, -0.607), (0.000, 0.075, -0.721)],
        
        "4AP": [(-0.766, 1.049, -0.529), (0.000, -0.099, -1.035)],
        "4": [(-0.766, 1.173, -0.431), (0.000, -0.099, -0.930)],
        
        "5AP": [(-0.661, 1.066, -0.396), (0.000, -0.099, -1.209)],
        "5": [(-0.668, 1.199, -0.274), (0.000, -0.274, -1.105)],
        
        "T1AP": [(-0.888, 1.129, -0.103), (0.000, -0.518, -0.930)],
        "T1": [(-0.888, 1.247, -0.016), (0.000, -0.518, -0.756)],
        
        "T2AP": [(-1.004, 1.246, 0.265), (0.000, -0.780, -0.756)],
        "T2": [(-1.004, 1.373, 0.363), (0.000, -0.780, -0.651)],
        
        "T3AP": [(-1.080, 1.456, 0.754), (0.000, -1.042, -0.581)],
        "T3": [(-1.080, 1.595, 0.925), (0.000, -1.077, -0.581)],
        
        "T4AP": [(-1.087, 1.045, -0.335), (0.000, -0.274, -0.581)],
        "T4": [(-1.087, 1.193, -0.234), (0.000, -0.379, -0.581)],
        
        "T5AP": [(-1.164, 1.167, 0.066), (0.000, -0.641, -0.581)],
        "T5": [(-1.164, 1.289, 0.169), (0.000, -0.641, -0.477)],
        
        "T6AP": [(-1.230, 1.359, 0.534), (0.000, -0.955, -0.477)],
        "T6": [(-1.221, 1.471, 0.639), (0.000, -0.955, -0.372)],
        
        "T7AP": [(-1.314, 1.045, -0.431), (0.000, -0.274, -0.232)],
        "T7": [(-1.314, 1.176, -0.339), (0.000, -0.274, -0.232)],
        
        "T8AP": [(-1.370, 1.145, -0.042), (0.000, -0.606, -0.302)],
        "T8": [(-1.370, 1.258, 0.045), (0.000, -0.606, -0.197)],
        
        "T9AP": [(-1.387, 1.284, 0.412), (0.000, -0.955, -0.302)],
        "T9": [(-1.387, 1.429, 0.520), (0.000, -0.955, -0.197)],
    "TOP_CENTER": [(-0.136, 1.018, 1.639), (0.000, -0.640, -0.581)]
        }

        self.reset_game()


        # Initial board display
        self.clear_terminal()
        self.display_board()
        print("\nWaiting for moves... (X: Robot, O: Human)")
        print("\nPress 'r' and Enter to restart the game at any time")

    def handle_keyboard_input(self):
        """Handle keyboard input for game reset"""
        while True:
            try:
                user_input = input().strip().lower()
                if user_input == 'r':
                    self.reset_game()
            except Exception:
                pass

    def reset_game(self):
        """Reset the game state"""
        self.board_state = [""] * 9
        self.committed_moves = [""] * 9
        self.last_detected_board = [""] * 9
        self.current_figure = 1
        self.waiting_for_human = True
        self.game_active = True
        self.clear_terminal()
        self.display_board()
        print("\nGame Reset! Waiting for human move...")        
        


        # Initial board display
        self.clear_terminal()
        self.display_board()
        print("\nWaiting for moves... (X: Robot, O: Human)")

    def find_best_move(self):
        """Find the best move for the robot using a simple strategy"""
        # Check if we can win in the next move
        for i in range(9):
            if self.board_state[i] == "":
                self.board_state[i] = 'X'
                if self.check_winner():
                    self.board_state[i] = ""
                    return i
                self.board_state[i] = ""

        # Check if opponent can win in their next move and block them
        for i in range(9):
            if self.board_state[i] == "":
                self.board_state[i] = 'O'
                if self.check_winner():
                    self.board_state[i] = ""
                    return i
                self.board_state[i] = ""

        # Take center if available
        if self.board_state[4] == "":
            return 4

        # Take corners if available
        corners = [0, 2, 6, 8]
        available_corners = [i for i in corners if self.board_state[i] == ""]
        if available_corners:
            return available_corners[0]

        # Take any available edge
        edges = [1, 3, 5, 7]
        available_edges = [i for i in edges if self.board_state[i] == ""]
        if available_edges:
            return available_edges[0]

        return None

    def bounding_boxes_callback(self, data):
        """Process all detected symbols and update the board state"""
        if not self.game_active or not self.waiting_for_human:
            return

        current_board = self.committed_moves.copy()  # Start with committed moves
        
        # Process all detected boxes
        for box in data.bounding_boxes:
            move_index = self.map_bounding_box_to_grid(box)
            if move_index is not None:  # Only process moves within boundaries
                if box.Class == "oxx" and self.committed_moves[move_index] == "":
                    current_board[move_index] = "O"
                elif box.Class == "ixx":
                    # For X moves, only show them if they're committed
                    if self.committed_moves[move_index] == "X":
                        current_board[move_index] = "X"
        
        # Check if a new valid O move has been made
        if current_board != self.last_detected_board:
            new_o_moves = [i for i in range(9) if current_board[i] == "O" and self.committed_moves[i] == ""]
            if len(new_o_moves) == 1:  # Exactly one new O move
                move_index = new_o_moves[0]
                self.committed_moves[move_index] = "O"  # Commit the move
                self.board_state = self.committed_moves.copy()
                self.last_detected_board = self.board_state.copy()
                self.waiting_for_human = False
                
                self.clear_terminal()
                self.display_board()
                
                if self.check_winner():
                    print("\nHuman wins!")
                    self.game_active = False
                    return
                
                # Make robot's move
                best_move = self.find_best_move()
                if best_move is not None:
                    self.committed_moves[best_move] = 'X'  # Commit the robot's move
                    self.board_state = self.committed_moves.copy()
                    self.pick_and_place(best_move)
                    self.current_figure += 1
                    
                    self.clear_terminal()
                    self.display_board()
                    
                if self.check_winner():
                    print("\nRobot wins!")
                    print(ROBOT_WINS)
                    self.game_active = False
                    return
                elif "" not in self.committed_moves:
                    print("\nGame Over - It's a tie!")
                    print(TIE_GAME)
                    self.game_active = False
                    return
        
                self.waiting_for_human = True
                self.display_board()





    def apply_joint_offsets(self, coords):
        """
        Modified to handle complete joint state
        Returns the complete position tuple structure
        """
        return coords  # Since we're using full joint states, we don't need to modify them


    def clear_terminal(self):
        print("\033[H\033[J", end="")

    def map_bounding_box_to_grid(self, box):
        """Maps the detected bounding box to a grid position."""
        # Calculate box center
        box_x_center = (box.xmin + box.xmax) / 2
        box_y_center = (box.ymin + box.ymax) / 2

        # Define the grid boundaries in pixel coordinates
        left_boundary = 197
        right_boundary = 598
        top_boundary = 25
        bottom_boundary = 431

        # Calculate relative position within grid
        x_pos = (box_x_center - left_boundary) / (right_boundary - left_boundary)
        y_pos = (box_y_center - top_boundary) / (bottom_boundary - top_boundary)

        # Convert to grid coordinates (0-2)
        grid_x = int(x_pos * 3)
        grid_y = int(y_pos * 3)

        # Return None if outside the game board boundaries
        if 0 <= grid_x < 3 and 0 <= grid_y < 3:
            return grid_y * 3 + grid_x
        return None



    def check_winner(self):
        """Check for a winner in any direction"""
        win_conditions = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],  # Rows
            [0, 3, 6], [1, 4, 7], [2, 5, 8],  # Columns
            [0, 4, 8], [2, 4, 6]              # Diagonals
        ]
        
        for condition in win_conditions:
            if (self.board_state[condition[0]] == self.board_state[condition[1]] == 
                self.board_state[condition[2]] != ""):
                return True
        return False

    def display_board(self):
        """Display the current game board in the terminal"""
        self.clear_terminal()
        print("\n╔═══╦═══╦═══╗")
        for i in range(3):
            row = self.committed_moves[i * 3:(i + 1) * 3]
            row = [' ' if x == '' else x for x in row]
            print(f"║ {row[0]} ║ {row[1]} ║ {row[2]} ║")
            if i < 2:
                print("╠═══╬═══╬═══╣")
        print("╚═══╩═══╩═══╝")
        
        # Print current state
        if not self.game_active:
            if self.check_winner():
                if self.current_turn == 'X':
                    print(HUMAN_WINS)
                else:
                    print(ROBOT_WINS)
            else:
                print(TIE_GAME)
            print("\nPress 'r' and Enter to restart")
        elif self.waiting_for_human:
            print("\nWaiting for human move...")
        else:
            print("\nRobot is thinking...")


    def move_to_position(self, position_key):
        """Move to a specific position with approach position handling"""
        try:
            print(f"\nMoving to position: {position_key}")
            
            # Move to approach position
            print(f"-> Approach position {position_key}AP")
            ap_position = self.positions[f"{position_key}AP"]  # Get complete position tuple
            self.move_to_coordinates(ap_position)
            
            # Move to target position
            print(f"-> Target position {position_key}")
            target_position = self.positions[position_key]  # Get complete position tuple
            self.move_to_coordinates(target_position)
            
            # Return to approach position
            print(f"-> Return to approach position {position_key}AP")
            self.move_to_coordinates(ap_position)
            
        except KeyError as e:
            print(f"\nERROR: Invalid position reference: {e}")
        except Exception as e:
            print(f"\nERROR during movement: {e}")



    def pick_and_place(self, target_position):
        """Modified pick_and_place to keep board visible"""
        try:
            figure_pos = str(self.current_figure)
            table_pos = f"T{target_position + 1}"

            def update_display(status):
                self.clear_terminal()
                self.display_board()
                print(f"\nRobot action: {status}")

            update_display(f"Starting pick and place sequence for figure {figure_pos} to position {table_pos}")
            
            # Pick sequence
            update_display("Opening gripper")
            self.command_publisher.publish("open_gripper")
            xogcode.main(None, self.serial_port, "open")
            
            update_display(f"Moving to approach position {figure_pos}AP")
            self.move_to_coordinates(self.positions[f"{figure_pos}AP"])
            xogcode.main(f"{figure_pos}AP", self.serial_port)
            
            # Continue with all movements, updating display each time
            update_display(f"Moving to pick position {figure_pos}")
            self.move_to_coordinates(self.positions[figure_pos])
            xogcode.main(figure_pos, self.serial_port)
            
            update_display("Closing gripper")
            self.command_publisher.publish("close_gripper")
            xogcode.main(None, self.serial_port, "close")
            
            update_display(f"Returning to approach position {figure_pos}AP")
            self.move_to_coordinates(self.positions[f"{figure_pos}AP"])
            xogcode.main(f"{figure_pos}AP", self.serial_port)
            
            update_display("Moving to top position")
            xogcode.main("top", self.serial_port)
            
            update_display(f"Moving to approach position {table_pos}AP")
            self.move_to_coordinates(self.positions[f"{table_pos}AP"])
            xogcode.main(f"{table_pos}AP", self.serial_port)
            
            update_display(f"Moving to place position {table_pos}")
            self.move_to_coordinates(self.positions[table_pos])
            xogcode.main(table_pos, self.serial_port)
            
            update_display("Opening gripper")
            self.command_publisher.publish("open_gripper")
            xogcode.main(None, self.serial_port, "open")
            
            update_display(f"Returning to approach position {table_pos}AP")
            self.move_to_coordinates(self.positions[f"{table_pos}AP"])
            xogcode.main(f"{table_pos}AP", self.serial_port)
            
            update_display("Moving to top position")
            xogcode.main("top", self.serial_port)
            
            update_display("Moving to park position")
            xogcode.main("park", self.serial_port)
            
        except Exception as e:
            print(f"Error during pick and place: {e}")


    def move_to_coordinates(self, coords):
        try:
            # Handle both single position and tuple of positions
            if isinstance(coords[0], tuple):
                # If it's a tuple of positions, use the first one
                j1, j2, j3 = coords[0]
                j4, j5, j6 = coords[1]
            else:
                # If it's a single position
                j1, j2, j3 = coords
                j4, j5, j6 = (0.0, 0.0, 0.0)  # Default values for other joints
                
            command = f"go_to_joint_state,{j1:.4f},{j2:.4f},{j3:.4f},{j4:.4f},{j5:.4f},{j6:.4f}"
            print("\n" + "="*50)
            print("SENDING MOVEMENT COMMAND:")
            print(f"Raw coordinates: {coords}")
            print(f"Command string: {command}")
            print("="*50 + "\n")
            
            self.command_publisher.publish(command)
            rospy.sleep(0.5)  # Give time for movement to complete
            
        except Exception as e:
            print(f"\nERROR during movement: {e}")
        


    def update_gripper_state(self, action):
        """Update the gripper state (open/close)"""
        return f"gripper_command,{'open' if action == 'open' else 'close'}"

if __name__ == "__main__":
    try:
        game = TicTacToe()
        game.waiting_for_human = True
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
