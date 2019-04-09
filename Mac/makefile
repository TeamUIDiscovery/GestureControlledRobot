#INCLUDE := /usr/local/include
#LEAP_LIBRARY := /usr/local/lib/libLeap.dylib
INCLUDE := ../include
LEAP_LIBRARY := ../lib/libLeap.dylib

BIN := main

CXX := g++
CXXFLAGS=-Wall -g -I   # debug
LIBS = -lm

SRCS=\
$(BIN).cpp\
listener.cpp

HDRS=\
listener.h

all: $(BIN) clean

$(BIN): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(SRCS) $(LIBS) -o $(BIN) $(LEAP_LIBRARY)

clean:
	rm -rf $(BIN).dSYM
clean-all:
	rm -rf $(BIN).dSYM $(BIN)