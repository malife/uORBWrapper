# Makefile for uORBWrapper test driving
#
# make             makes QueueClient
# make clean       removes object files
#

CC=gcc
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`
SRC_PUB=uORB.c publisher.c random_integer.c 
SRC_LIS=uORB.c random_integer.c subscriber.c
OBJ_PUB=$(SRC_PUB:.c=.o)
OBJ_LIS=$(SRC_LIS:.c=.o)
PUB=publisher
LIS=listener

all: $(PUB) $(LIS)

$(PUB): $(OBJ_PUB)
	$(CC) $(OBJ_PUB) -o $(PUB) $(LDFLAGS)

$(LIS): $(OBJ_LIS)
	$(CC) $(OBJ_LIS) -o $(LIS) $(LDFLAGS)

uORB.o: uORB.h
	$(CC) -c -DNOT_NUTTX $(CFLAGS) uORB.c

random_integer.o: random_integer.h
	$(CC) -c -DNOT_NUTTX $(CFLAGS) random_integer.c

publisher.o:
	$(CC) -c -DNOT_NUTTX  $(CFLAGS) publisher.c

subscriber.o:
	$(CC) -c -DNOT_NUTTX  $(CFLAGS) subscriber.c

clean:
	rm -f *o publisher listener