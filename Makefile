BIN=pathfind

$(BIN): *.cpp *.hpp
	g++ -o $@ *.cpp -lm
