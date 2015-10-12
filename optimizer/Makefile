OR_TOOLS_TOP=../../or-tools/or-tools
OR_TOOLS_SOURCES=$(OR_TOOLS_TOP)/src

TUTORIAL=../../or-tools/or-tools/documentation/tutorials/cplusplus


include $(OR_TOOLS_TOP)/Makefile

.PHONY: all local_clean

all: $(EXE)

tsp_simple.o: tsp_simple.cc $(OR_TOOLS_SOURCES)/constraint_solver/routing.h \
	$(TUTORIAL)/routing_common/routing_common.h \
	$(TUTORIAL)/routing_common/routing_data.h \
	tsptw_data_dt.h \
	$(TUTORIAL)/routing_common/tsplib.h tsptw_solution_dt.h
	$(CCC) $(CFLAGS) -I $(TUTORIAL) -c tsp_simple.cc -o tsp_simple.o

tsp_simple: $(ROUTING_DEPS) tsp_simple.o
	$(CCC) $(CFLAGS) -g tsp_simple.o $(DYNAMIC_ROUTING_LNK) $(DYNAMIC_LD_FLAGS) -o tsp_simple

local_clean:
	rm *.o
