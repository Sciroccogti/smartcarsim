PROJ = ai
SCS = libscs
SCS_SRC_DIR = ../../scs/src
SCS_INC_DIR = ../../scs/include
AI_SRC_DIR = ../src
AI_INC_DIR = ../include
VPATH = $(SCS_SRC_DIR):$(SCS_INC_DIR):$(AI_SRC_DIR):$(AI_INC_DIR)

CXX = g++
CXXFLAGS = -I$(SCS_INC_DIR) -I$(AI_INC_DIR) -g -DdDOUBLE
SCSOBJ = simulation.o draw.o car.o track.o api.o vector.o ui.o
AIOBJ = main.o
LIB = -lscs -lode -lglut -lGLU -lm -lGL -lz -lrt -lm -lpthread

all: $(SCS).a $(PROJ) run

$(SCS).a: $(SCSOBJ)
	ar rcs $@ $^

$(PROJ): $(AIOBJ) $(SCS).a
	$(CXX) -o $@ $< -L./ $(LIB)

run:
	./$(PROJ)

main.o: scs.h
simulation.o: car.h track.h draw.h api.h vector.h simulation.h
draw.o: mytime.h simulation.h key.h car.h track.h common.h api.h draw.h
car.o: car.h draw.h vector.h 
api.o: simulation.h draw.h common.h car.h mytime.h api.h vector.h track.h
vector.o: vector.h common.h
track.o: track.h common.h car.h draw.h vector.h
ui.o: ui.h draw.h car.h key.h api.h simulation.h

clean:
	$(RM) *.o
	$(RM) $(SCS).a
	$(RM) $(PROJ)
