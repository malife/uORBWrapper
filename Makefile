# Makefile for uORBWrapper test driving
#
# make             makes QueueClient
# make clean       removes object files
#

CC=gcc
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`
SOURCES=uORB.c publisher.c random_integer.c
OBJECTS=$(SOURCES:.c=.o)
EXE=publisher

$(EXE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXE) $(LDFLAGS)

uORB.o: uORB.h
	$(CC) -c -DNOT_NUTTX $(CFLAGS) uORB.c

random_integer.o: random_integer.h
	$(CC) -c -DNOT_NUTTX $(CFLAGS) random_integer.c

publisher.o:
	$(CC) -c -DNOT_NUTTX  $(CFLAGS) publisher.c

clean:
	rm -f *o publisher