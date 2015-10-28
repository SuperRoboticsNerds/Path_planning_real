# Define compiler
CC = g++
# External libraries to include
LDLIBS = 
# Header files to include
INCLUDES = 
# Flags for compiler
CCFLAGS = -Wall
# Flags for linker
LDFLAGS = 

# The rule for will be automatically generated
solvePlanningProblem: solvePlanningProblem.cc AStarNode.cc Position.cc PRM.cc PRMNode.cc MyWorld.cc circle.cc rectangle.cc World.cc obstacle.cc

clean:
	rm -f *.o solvePlanningProblem
