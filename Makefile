OR_TOOLS_TOP=../or-tools
OR_TOOLS_SOURCES=$(OR_TOOLS_TOP)/ortools

TUTORIAL=resources


include $(OR_TOOLS_TOP)/Makefile

.PHONY: all local_clean

all: $(EXE)

%.pb.cc: %.proto
	$(OR_TOOLS_TOP)/dependencies/install/bin/protoc --cpp_out . $<

%.o: %.cc %.h
	$(CCC) $(CFLAGS) -c $< -o $@

ortools_vrp.pb.h: ortools_vrp.pb.cc

tsp_simple.o: tsp_simple.cc $(OR_TOOLS_SOURCES)/constraint_solver/routing.h ortools_vrp.pb.h \
	$(TUTORIAL)/routing_common/routing_common.h \
	tsptw_data_dt.h \
	limits.h
	$(CCC) $(CFLAGS) -I $(TUTORIAL) -c tsp_simple.cc -o tsp_simple.o

tsp_simple: $(ROUTING_DEPS) tsp_simple.o ortools_vrp.pb.o $(OR_TOOLS_TOP)/lib/libortools.so
	$(CCC) $(CFLAGS) -g tsp_simple.o ortools_vrp.pb.o $(OR_TOOLS_LD_FLAGS) \
	-L $(OR_TOOLS_TOP)/lib -Wl,-rpath $(OR_TOOLS_TOP)/lib -lcvrptw_lib -ldimacs -lortools -L $(OR_TOOLS_TOP)/dependencies/install/lib -lprotobuf \
	-o tsp_simple

local_clean:
	rm -f *.pb.cc *.pb.h
	rm *.o

mrproper: local_clean
	rm tsp_simple
