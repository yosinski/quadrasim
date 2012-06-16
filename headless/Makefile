# MAKEFILE TO COMPILE THE QUADRATOT SIMULATOR
CC = g++
PREPROCESSOR = -DLINUX -DNX32 -DCORELIB -DNX_DISABLE_FLUIDS
physXPath = $(shell ls -d  /usr/include/PhysX/v*)
physXLibPath = $(shell ls -d /usr/lib/PhysX/v*)
INCLUDES = -I. -Ibase -Ihardware -Imachines -Iphysx -I/usr/include/boost -I$(physXPath)/SDKs/Physics/include -I$(physXPath)/LowLevel/API/include -I$(physXPath)/LowLevel/hlcommon/include -I$(physXPath)/SDKs/Cooking/include -I$(physXPath)/SDKs/PhysXLoader/include -I$(physXPath)/SDKs/Foundation/include -I$(physXPath)/SDKs/NxCharacter/include
CPPFLAGS= $(PREPROCESSOR) $(INCLUDES) -Wall

LDIR = -L$(physXLibPath) -L/home/sean/Downloads/dyna/DXL_SDK_LINUX_v1_01/src
LIBS = -ldl -ldxl -lNxCharacter -lNxCooking -lPhysXCore -lPhysXLoader -ljpeg -lpthread

OBJ = main.o part.o physics.o testobjects.o base/Commandline.o base/floatparammultivallist.o base/Vec3.o base/floatparamlist.o base/kmath.o base/tools.o base/Wingetopt.o hardware/axservo.o hardware/mocap.o hardware/quadrohardware.o machines/quadromachine.o machines/quadrobot.o physx/cooking.o physx/Stream.o physx/VertexWelder.o physx/wavefront.o

all: quadmake

%.o: %.c
	$(CC) -c $(CPPFLAGS) $< -o $@

%.o: %.cpp
	$(CC) -c $(CPPFLAGS) $< -o $@

quadmake: $(OBJ)
	$(CC) $(CPPFLAGS) -o quadmake $(OBJ) $(LIBS) $(LDIR)

clean:
	rm -f $(OBJ) quadmake
