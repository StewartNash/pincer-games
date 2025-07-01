#!/bin/bash

OUTPUT="program"

CXX=g++
CXXFLAGS="-std=c++17 -Wall -Wextra -O2"

SOURCES="main.cpp checkers.cpp"

$CXX $CXXFLAGS $SOURCES -o $OUTPUT

if [ $? -eq 0 ]; then
	echo "Build successful. Executable is ./$OUTPUT"
else
	echo "Build failed."
fi
