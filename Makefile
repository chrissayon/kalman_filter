CC=g++
CFLAGS=-Wall 
SRC=./src
INCLUDE=./include

all: main test
# all: $(SRC)/%.o

main: main_compile kalman_compile
	$(CC) main.o kalman_filter.o -o kalman_build.exe

main_compile: 
	$(CC) $(CFLAGS) -c $(SRC)/main.cpp -I$(INCLUDE)

kalman_compile:
	$(CC) $(CFLAGS) -c $(SRC)/kalman_filter.cpp -I$(INCLUDE)

# $(SRC)/%.o: $(SRC)/%.cpp 
# 	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o

test:
	cd tests && $(MAKE)