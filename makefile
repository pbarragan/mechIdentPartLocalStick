UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
all:
	g++ -g *.cpp -lrt -o testRealWorld
endif
ifeq ($(UNAME), Darwin)
all:
	g++ -g *.cpp -o testRealWorld
endif
