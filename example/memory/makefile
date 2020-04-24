CC=g++
CFLAGS = -std=c++11
LIBS=-lpthread -lpid_sgd
LIB_PATH=
LIB_PATH+=-L../../lib
INCLUDE_PATH=
INCLUDE_PATH+=-I../..

OBJ =	memory.o \
	

%.o: %.cpp
	$(CC) -g -c -o $@  $< $(CFLAGS) $(INCLUDE_PATH)

memory: memory.o
	$(CC) -g memory.o -o $@ $(CFLAGS) $(LIBS) $(LIB_PATH)

clean:
	rm -f *.o memory
