

CC	= g++

OBJS	= ops_pedestrian_system.o

CFLAGS	= -c -fPIC -g -O2 -m64

INC_FLAGS = -I/opt/houdini/toolkit/include -I/home/jaideep/opensteer/include -I/home/jaideep/workspaceB/ops/lib -I/home/jaideep/workspaceB/ops/houdini/lib

WFLAGS	= -Wall -W -Wno-parentheses -Wno-sign-compare -Wno-reorder -Wno-uninitialized -Wunused -Wno-unused-parameter -Wno-deprecated

DFLAGS	= -DVERSION=\"11.1.67\" -D_GNU_SOURCE -DLINUX -DAMD64 -DSIZEOF_VOID_P=8 -DSESI_LITTLE_ENDIAN -DENABLE_THREADS -DUSE_PTHREADS -D_REENTRANT \
-D_FILE_OFFSET_BITS=64 -DGCC4 -DGCC3 -DNEED_SPECIALIZATION_STORAGE -DMAKING_DSO

all : $(OBJS)

ops_pedestrian_system.o : ops_pedestrian_system.cpp
	$(CC) $^ $(CFLAGS) $(INC_FLAGS) $(WFLAGS) $(DFLAGS) -o $@

clean :
	rm $(OBJS)

