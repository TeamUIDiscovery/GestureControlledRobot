INCLUDE := /usr/local/include
LEAP_LIBRARY := /usr/local/lib/libLeap.dylib

all: hello clean

hello: leapMotion.cpp sampleListener.cpp
	$(CXX) -Wall -g -I $(INCLUDE) leapMotion.cpp sampleListener.cpp -o hello $(LEAP_LIBRARY)

clean:
	rm -rf helloWorld.dSYM