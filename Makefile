OR_TOOLS_TOP=../or-tools
OR_TOOLS_SOURCES=$(OR_TOOLS_TOP)/ortools

TUTORIAL=resources


include $(OR_TOOLS_TOP)/Makefile

# For debugging uncomment the next line. -isystem prevents warnings rooted in or-tools library appearing in our compilation
# CFLAGS := $(CFLAGS) -ggdb -Og -DDEBUG -fsanitize=address -Wall -Wextra -Wshadow -Wunreachable-code -Winit-self -Wmissing-include-dirs -Wswitch-enum -Wfloat-equal -Wundef -isystem$(OR_TOOLS_TOP)/. -isystem$(OR_TOOLS_SOURCES)/gen -isystem$(OR_TOOLS_TOP)/dependencies/install/include -isystem$(OR_TOOLS_TOP)/dependencies/install/include/coin -DUSE_CBC -DUSE_CLP -DUSE_GLOP -DUSE_BOP

.PHONY: all local_clean

all: $(EXE)

%.pb.cc: %.proto
	$(OR_TOOLS_TOP)/dependencies/install/bin/protoc --cpp_out . $<

%.o: %.cc %.h
	$(CCC) $(CFLAGS) -c $< -o $@

ortools_vrp.pb.h: ortools_vrp.pb.cc

ortools_result.pb.h: ortools_result.pb.cc

tsp_simple.o: tsp_simple.cc $(OR_TOOLS_SOURCES)/constraint_solver/routing.h ortools_vrp.pb.h \
	ortools_result.pb.h \
	$(TUTORIAL)/routing_common/routing_common.h \
	tsptw_data_dt.h \
	limits.h
	$(CCC) $(CFLAGS) -I $(TUTORIAL) -c tsp_simple.cc -o tsp_simple.o

tsp_simple: $(ROUTING_DEPS) tsp_simple.o ortools_vrp.pb.o ortools_result.pb.o $(OR_TOOLS_TOP)/lib/libortools.so
	$(CCC) $(CFLAGS) -g tsp_simple.o ortools_vrp.pb.o ortools_result.pb.o $(OR_TOOLS_LD_FLAGS) \
	-L $(OR_TOOLS_TOP)/lib -Wl,-rpath $(OR_TOOLS_TOP)/lib -lortools -L $(OR_TOOLS_TOP)/dependencies/install/lib -lprotobuf -lglog -lgflags \
	-o tsp_simple

local_clean:
	rm -f *.pb.cc *.pb.h *.o

mrproper: local_clean
	rm -f tsp_simple
