CC=g++
CFLAGS=-std=c++11 -Wall 

all: tests.o catch.hpp.gch
	$(CC) $(CFLAGS) tests.o -include catch.hpp -o kalman.exe 

tests.o: tests.cpp
	$(CC) $(CFLAGS) -c tests.cpp

catch.hpp.gch: catch.hpp
	$(CC) $(CFLAGS) -c catch.hpp
