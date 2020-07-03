CC=g++
CFLAGS=-std=c++11 -Wall
INCLUDE=../include
LIBS=../libs/Eigen
SRC=../src

all: tests.o catch.hpp.gch kalman_filter.o
	$(CC) $(CFLAGS) tests.o kalman_filter.o -include catch.hpp.gch -o kalman_test.exe 

tests.o: tests.cpp
	$(CC) $(CFLAGS) -c tests.cpp -I$(INCLUDE) -I$(LIBS) -I$(SRC) 

kalman_filter.o:
	$(CC) $(CFLAGS) -c $(SRC)/kalman_filter.cpp -I$(INCLUDE) -I$(LIBS) -I$(SRC) 

catch.hpp.gch: catch.hpp
	$(CC) $(CFLAGS) -c catch.hpp
