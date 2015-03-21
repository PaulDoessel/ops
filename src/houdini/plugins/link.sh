
g++ -shared ops_pedestrian_system.o /home/jaideep/workspaceB/ops/houdini/lib/houdini_ops/*.o /home/jaideep/workspaceB/ops/lib/ops/*.o /home/jaideep/opensteer/linux/objs_optimized/*.o -L/usr/X11R6/lib64 -L/usr/X11R6/lib -lGLU -lGL -lX11 -lXext -lXi -ldl -o /home/jaideep/houdini11.1/dso/ops_pedestrian_system.so

