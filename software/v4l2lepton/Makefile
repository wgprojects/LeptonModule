
CC = gcc
CXX = g++
CFLAGS        = -O0 -g -pipe -Wall -W -D_REENTRANT -lpthread -lLEPTON_SDK -L/usr/lib/arm-linux-gnueabihf -L../raspberrypi_libs/leptonSDKEmb32PUB/Debug
CXXFLAGS      = -O0 -g -pipe -Wall -W -D_REENTRANT -lpthread -lLEPTON_SDK -L/usr/lib/arm-linux-gnueabihf -L../raspberrypi_libs/leptonSDKEmb32PUB/Debug
INCPATH = -I. -I../raspberrypi_libs -I../raspberrypi_libs/leptonSDKEmb32PUB 

all: sdk leptsci.o SPI.o Lepton_I2C.o Palettes.o v4l2lepton.o v4l2lepton

sdk:
	make -C ../raspberrypi_libs/leptonSDKEmb32PUB

sdkclean:
	make -C ../raspberrypi_libs/leptonSDKEmb32PUB clean

Palettes.o: Palettes.cpp Palettes.h
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o Palettes.o Palettes.cpp

SPI.o: SPI.cpp SPI.h
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o SPI.o SPI.cpp

Lepton_I2C.o: 
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o Lepton_I2C.o Lepton_I2C.cpp

v4l2lepton.o: leptsci.o Palettes.o SPI.o
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o v4l2lepton leptsci.o Palettes.o SPI.o v4l2lepton.cpp

v4l2lepton: v4l2lepton.o leptsci.o Palettes.o SPI.o
	${CXX} ${INCPATH} -o v4l2lepton leptsci.o Palettes.o SPI.o v4l2lepton.cpp ${CXXFLAGS}

leptsci.o: leptsci.c

clean:
	rm -f SPI.o Lepton_I2C.o Palettes.o leptsci.o v4l2lepton.o v4l2lepton
