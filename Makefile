BIN=pathfind

OBJ=problem.o ui.o main.o

$(BIN): *.hpp $(OBJ)
	g++ -o $@ $(OBJ) -lm -lsfml-system -lsfml-window -lsfml-graphics -O2 -std=c++11

%.o: %.cpp *.hpp
	g++ -c -o $@ $< -std=c++11 -O2
