BIN=pathfind

$(BIN): *.cpp *.hpp
	g++ -o $@ *.cpp -lm -O2 -std=c++11
