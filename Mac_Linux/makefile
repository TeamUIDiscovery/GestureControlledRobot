OS := $(shell uname)
ARCH := $(shell uname -m)

ifeq ($(OS), Linux)
  ifeq ($(ARCH), x86_64)
    LEAP_LIBRARY := lib/x64/libLeap.so -Wl,-rpath,../lib/x64
  else
    LEAP_LIBRARY := lib/x86/libLeap.so -Wl,-rpath,../lib/x86
  endif
endif
ifeq ($(OS), Darwin)
  LEAP_LIBRARY := lib/libLeap.dylib
else
  $(error Operating System not OS X or Linux)
endif

INCLUDE := include

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