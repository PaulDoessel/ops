

CC	= g++

OBJS	= pedestrian.o steer_world.o

CFLAGS	= -c -fPIC -I/home/jaideep/opensteer/include

all : $(OBJS)

pedestrian.o : pedestrian.cpp
	$(CC) $^ $(CFLAGS) -o $@

steer_world.o : steer_world.cpp
	$(CC) $^ $(CFLAGS) -o $@

clean :
	rm $(OBJS)

