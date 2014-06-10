# Makefile for uORBWrapper test driving
#
# make             makes QueueClient
# make clean       removes object files
#

CC=gcc
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`
SRC_PUB=uORB.c publisher.c random_integer.c 
SRC_SUB=uORB.c random_integer.c subscriber.c
OBJ_PUB=$(SRC_PUB:.c=.o)
OBJ_SUB=$(SRC_SUB:.c=.o)
PUB=publisher
SUB=subscriber

all: $(PUB) $(SUB)

$(PUB): $(OBJ_PUB)
	$(CC) $(OBJ_PUB) -o $(PUB) $(LDFLAGS)

$(SUB): $(OBJ_SUB)
	$(CC) $(OBJ_SUB) -o $(SUB) $(LDFLAGS)

uORB.o: uORB.h
	$(CC) -c -DNOT_NUTTX $(CFLAGS) uORB.c

random_integer.o: random_integer.h
	$(CC) -c -DNOT_NUTTX $(CFLAGS) random_integer.c

publisher.o:
	$(CC) -c -DNOT_NUTTX  $(CFLAGS) publisher.c

subscriber.o:
	$(CC) -c -DNOT_NUTTX  $(CFLAGS) subscriber.c

clean:
	rm -f *o publisher subscriber