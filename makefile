UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
all:
	g++ *.cpp -lrt -o testRealWorld.o
endif
ifeq ($(UNAME), Darwin)
all:
	g++ *.cpp -o testRealWorld.o
endif
