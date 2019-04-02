INCLUDE := /usr/local/include
LEAP_LIBRARY := /usr/local/lib/libLeap.dylib

all: leap clean

leap: leapMotion.cpp listener.cpp
	$(CXX) -Wall -g -I $(INCLUDE) leapMotion.cpp listener.cpp -o leap $(LEAP_LIBRARY)

clean:
	rm -rf helloWorld.dSYM