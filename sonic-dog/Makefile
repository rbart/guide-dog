CC= g++
CFLAGS= -Wall -g -I/opt/openal/include -L/opt/openal/lib
OBJS= test.o sim.o sonic_dog.o
PROGRAMS = sim

sonic: sonic_dog.cpp sonic_dog.h
	$(CC) $(CFLAGS) -c sonic_dog.cpp

sim: sim.o sonic_dog.o
	$(CC) $(CFLAGS) sim.o sonic_dog.o -o sim -lopenal -lalut -ltinfo -lncurses -lpthread

test: test.o 
	$(CC) $(CFLAGS) test.o -o test -lopenal -lalut

all: sim sonic

%.o : %.cpp 
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f ${OBJS} ${PROGRAMS}
