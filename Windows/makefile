ifeq ($(ARCHITECTURE), Windows_NT)
  ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
    LEAP_LIBRARY := lib/x64/Leap.lib
  endif
  ifeq ($(PROCESSOR_ARCHITECTURE),x86)
    LEAP_LIBRARY := lib/x86/Leap.lib
  endif
else
  $(error Operating System not Windows)
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