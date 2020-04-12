CC=g++
CFLAGS = -std=c++11
LIBS=-lpthread

OBJ = memory.o

%.o: %.cpp
	$(CC) -c -o $@  $< $(CFLAGS)

memory: memory.o
	$(CC) memory.o -o $@ $(CFLAGS) $(LIBS)

clean:
	rm -f *.o memory
