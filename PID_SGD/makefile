CC=g++
CFLAGS = -std=c++11
LIBS=
LIBS+=-lpthread

OBJ =	PID_SGD.o \
	

%.o: %.cpp
	$(CC) -fPIC -g -c -o $@  $< $(CFLAGS)

libpid_sgd.so: PID_SGD.o
	ld -shared -g PID_SGD.o -o $@ $(LIBS)
	cp $@ ../lib

clean:
	rm -f *.o *.so
