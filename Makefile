
CXX:=clang++
SHAREDCFLAGS:=-O3 -ffast-math -fno-trapping-math -fno-math-errno -std=c++11 -funroll-loops -march=native -Wall -Wextra -pedantic -D_FILE_OFFSET_BITS=64 -x c++
SHAREDLDFLAGS:=-lfftw3 -lm -O3 -ffast-math -fno-trapping-math -fno-math-errno -std=c++11 -march=native -Wall -Wextra -pedantic -D_FILE_OFFSET_BITS=64

SRCDIR = src/
INCDIR = include/

SRC := $(wildcard $(SRCDIR)*.cpp)
DEPS:= $(wildcard $(INCDIR)*.h)
OBJ  = $(SRC:.cpp=.o)


all: CXXFLAGS:=-pthread $(SHAREDCFLAGS) -I$(INCDIR)
all: LDFLAGS:=-lpthread $(SHAREDLDFLAGS)
all: $(OBJ)
	$(CXX) $(LDFLAGS) -o yagpsr $^
	

nomultithread: CXXFLAGS:=-DD_YAGPSR_DISABLE_MULTITHREADING $(SHAREDCFLAGS) -I$(INCDIR)
nomultithread: LDFLAGS:=-DD_YAGPSR_DISABLE_MULTITHREADING $(SHAREDLDFLAGS)
nomultithread: $(OBJ)
	$(CXX) $(LDFLAGS) -o yagpsr $^		


.PHONY: clean cleanall
clean:
	rm $(OBJ)


cleanall:
	rm yagpsr $(OBJ)
