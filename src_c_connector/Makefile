# When clock skew appears uncomment this line
# $(shell find . -exec touch {} \;)

TARGET_EXEC := c_robotMPC.out

CC = gcc
CXX = g++
CXXFLAGS = -Wall -O1 $(shell pkg-config --cflags eigen3) -I../src

all :	main

main : main.o libRobotMPC_c_connector.a libRobotMPC.a
	$(CC) main.c -L. -lRobotMPC_c_connector -o $(TARGET_EXEC)

libRobotMPC_c_connector.a :	RobotMPC_c_connector.cpp libRobotMPC.a
	$(CXX) -fpic -shared RobotMPC_c_connector.cpp -L. -lRobotMPC -o libRobotMPC_c_connector.a $(CXXFLAGS)

libRobotMPC.a :	../src/RobotMPC.cpp
	$(CXX) -fpic -shared ../src/RobotMPC.cpp -o libRobotMPC.a $(CXXFLAGS)

mrc :	main
	mkdir -p ../../mrc/
	cp libRobotMPC.a ../../mrc/
	cp libRobotMPC_c_connector.a ../../mrc/

clean :	
	rm -f *.a *.o $(TARGET_EXEC)
	rm -f ../../mrc/libRobotMPC*.a
# Compile With
# g++ -fpic -shared aaa_c_connector.cpp -L. -laaa -o libaaa_c_connector.so -I/usr/local/Cellar/eigen/3.3.4/include/eigen3
# g++ -fpic -shared aaa.cpp -o libaaa.so -I/usr/local/Cellar/eigen/3.3.4/include/eigen3