CC=g++
CFLAGS=-Wall
SRC=./src

all: main test

# -o used to compile and link the file with executable file
main: main_compile
	$(CC) main.o -o kalman_test.exe

# -c used to compile and assemble the file and not link object code to produce executable file
main_compile: 
	$(CC) $(CFLAGS) -c $(SRC)/main.cpp

# Remove executable files
clean:
	rm -f *.o

# Unit tests
test:
	cd tests && $(MAKE)
	