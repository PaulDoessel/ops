
#!/bin/bash

#g++ steer_world_adapter.cpp -DMAKING_DSO -D_GNU_SOURCE -DLINUX -DAMD64 -DENABLE_THREADS -DUSE_PTHREADS -D_REENTRANT -D_FILE_OFFSET_BITS=64 -DGCC4 -DGCC3 -Wno-deprecated -DNEED_SPECIALIZATION_STORAGE -c -I/#opt/houdini/toolkit/include -I/home/jaideep/opensteer/include -I/home/jaideep/workspaceB/ops/lib -fPIC -o steer_world_adapter.o


#g++ -#DUT_DSO_TAGINFO='"3262197cbf18481d27a30898673090f52f4bd0bdda20ccaa27e3dd0609148baf34933589d3772d340c08b7a867a964524b84a05cefd0eb44420e27e964f090ba5bac55b1edb84994a3268d0847ba11bef254d5f085e39060f6b34d72"' -#DVERSION=\"11.1.67\" -D_GNU_SOURCE -DLINUX -DAMD64 -m64 -fPIC -DSIZEOF_VOID_P=8 -DSESI_LITTLE_ENDIAN -DENABLE_THREADS -DUSE_PTHREADS -D_REENTRANT -D_FILE_OFFSET_BITS=64 -c  -DGCC4 -DGCC3 -Wno-deprecated -#DNEED_SPECIALIZATION_STORAGE -I/opt/houdini/toolkit/include -I/home/jaideep/opensteer/include -I/home/jaideep/workspaceB/ops/lib -I/home/jaideep/workspaceB/ops/houdini/lib -Wall -W -Wno-parentheses -Wno-#sign-compare -Wno-reorder -Wno-uninitialized -Wunused -Wno-unused-parameter -O2 -DMAKING_DSO -g  -o steer_world_adapter.o steer_world_adapter.cpp


g++ -DVERSION=\"11.1.67\" -D_GNU_SOURCE -DLINUX -DAMD64 -m64 -fPIC -DSIZEOF_VOID_P=8 -DSESI_LITTLE_ENDIAN -DENABLE_THREADS -DUSE_PTHREADS -D_REENTRANT -D_FILE_OFFSET_BITS=64 -c  -DGCC4 -DGCC3 -Wno-deprecated -DNEED_SPECIALIZATION_STORAGE -I/opt/houdini/toolkit/include -I/home/jaideep/opensteer/include -I/home/jaideep/workspaceB/ops/lib -I/home/jaideep/workspaceB/ops/houdini/lib -Wall -W -Wno-parentheses -Wno-sign-compare -Wno-reorder -Wno-uninitialized -Wunused -Wno-unused-parameter -O2 -DMAKING_DSO -g  -o steer_world_adapter.o steer_world_adapter.cpp

