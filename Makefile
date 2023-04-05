CC=g++
CFLAGS=-Wall -g -I./gnuplot-iostream
LDFLAGS=-lboost_iostreams
JSONLD=-ljsoncpp
JSONHD=-I /usr/local/include
LIBS=$(JSONLD)
EXENAME=crowdsim

all: Agent.o CrowdObject.o Vector.o Wall.o CrowdWorld.o
	$(CC) $(CFLAGS) main.cpp *.o $(LDFLAGS) $(LIBS) -o $(EXENAME)

Agent.o: Agent.cpp
	$(CC) $(CFLAGS) -c Agent.cpp

CrowdObject.o: CrowdObject.cpp
	$(CC) $(CFLAGS) -c CrowdObject.cpp

CrowdWorld.o : CrowdWorld.cpp
	$(CC) $(CFLAGS) -c CrowdWorld.cpp

Vector.o : vector.cpp
	$(CC) $(CFLAGS) -c vector.cpp

Wall.o : Wall.cpp
	$(CC) $(CFLAGS) -c Wall.cpp

clean:
	rm -f *.o *~ *.out $(EXENAME)
