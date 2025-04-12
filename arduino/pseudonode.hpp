/*
 * File: pseudonode.hpp
 * Author: Stewart Nash
 * Date: April 12, 2025
 * Description: Replace ROS functionality with simpler serial
 * implementation.
 *
 */
#pragma once

#include <vector>
#include <string>

namespace pseudoros {

struct ArmJoint {
	int position1;
	int position2;
	int position3;
	int position4;
	int position5;
	int position6;
	int position7;
};

class PseudoNode {
	public:
		PseudoNode(int* joint_step);
		ArmJoint armSteps;
		int jointStatus;

		void enqueueMessage(String inputString);
		void spinOnce();
		static std::vector<std::string> split(const std::string& s, char separator);
	private:
		char* messageQueue;
		int* jointStep;

		void armCallback();
		void gripperCallback();
		void parseCommand(std::string command);
};

PseudoNode::PseudoNode(int* joint_step) {
	jointStep = joint_step;	
}

void PseudoNode::armCallback() {
	jointStatus = 1;
	jointStep[0] = armSteps.position1;
	jointStep[1] = armSteps.position2;
	jointStep[2] = armSteps.position2;
	jointStep[3] = armSteps.position2;
	jointStep[4] = armSteps.position2;
	jointStep[5] = armSteps.position2;
	jointStep[6] = armSteps.position2;
}

void PseudoNode::enqueueMessage(String inputString) {

}

void PseudoNode::gripperCallback() {

}

void PseudoNode::parseCommand(std::string command) {
	std::vector<std::string> partition;
	std::string token;
	double x, y, z;

}

void PseudoNode::spinOnce() {

}

std::vector<std::string> PseudoNode::split(const std::string& s, char separator) {
	std::vector<std::string> output;
	std::string::size_type prev_pos = 0, pos = 0;

	while((pose = s.find(separator, pos)) != std::string::npos) {
		std::string substring(s.substr(prev_pos, pos - prev_pos));
		output.push_back(substring);
		prev_pos = ++pos;
	}
	output.push_back(s.substr(prev_pos, pos - prev_pos));

	return output;
}

} /* namespace pseudoros */
