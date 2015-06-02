UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
all:
	g++ *.cpp ../translator.cpp ../latch1.cpp ../logUtils.cpp -lrt -o transExternal.o
endif
ifeq ($(UNAME), Darwin)
all:
	g++ *.cpp ../translator.cpp ../latch1.cpp ../logUtils.cpp -o transExternal.o
endif