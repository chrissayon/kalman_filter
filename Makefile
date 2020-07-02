
all: main_link

# -o used to compile and link the file with executable file
main_link: main_build
	g++ main.o -o kalman_test.exe

# -c used to compile and assemble the file and not link object code to produce executable file
main_build: 
	g++ -c main.cpp

# Remove executable files
clean:
	rm -f *.o