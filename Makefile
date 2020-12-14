OR_TOOLS_TOP=../or-tools

TUTORIAL=resources

CFLAGS := -std=c++11 -I $(OR_TOOLS_TOP)/include

# During development uncomment the next line to have debug checks and other verifications
# DEVELOPMENT = true
ifeq ($(DEVELOPMENT), true)
	# -isystem prevents warnings rooted in or-tools library appearing in our compilation
  CFLAGS := $(CFLAGS) -O0 -DDEBUG -ggdb3 -fsanitize=address -fkeep-inline-functions -fno-inline-small-functions -Wall -Wextra -Wshadow -Wunreachable-code -Winit-self -Wmissing-include-dirs -Wswitch-enum -Wfloat-equal -Wundef -isystem$(OR_TOOLS_TOP)/. -isystem$(OR_TOOLS_TOP)/include
  CXX := LSAN_OPTION=verbosity=1:log_threads=1 $(CXX)
else
  CFLAGS := $(CFLAGS) -O4 -DNDEBUG
endif

.PHONY: all local_clean

all: $(EXE)

%.pb.cc: %.proto
	$(OR_TOOLS_TOP)/bin/protoc --cpp_out . $<

%.o: %.cc %.h
	$(CXX) $(CFLAGS) -c ./$< -o $@

ortools_vrp.pb.h: ortools_vrp.pb.cc

ortools_result.pb.h: ortools_result.pb.cc

tsp_simple.o: tsp_simple.cc ortools_vrp.pb.h \
	ortools_result.pb.h \
	$(TUTORIAL)/routing_common/routing_common.h \
	tsptw_data_dt.h \
	limits.h \
	values.h
	$(CXX) $(CFLAGS) -I $(TUTORIAL) -c ./tsp_simple.cc -o tsp_simple.o

tsp_simple: $(ROUTING_DEPS) tsp_simple.o ortools_vrp.pb.o ortools_result.pb.o $(OR_TOOLS_TOP)/lib/libortools.so
	$(CXX) $(CFLAGS) -g tsp_simple.o ortools_vrp.pb.o ortools_result.pb.o $(OR_TOOLS_LD_FLAGS) \
	-L $(OR_TOOLS_TOP)/lib -Wl,-rpath $(OR_TOOLS_TOP)/lib -lortools -lprotobuf -lglog -lgflags -labsl_raw_hash_set -labsl_time -labsl_time_zone \
	-o tsp_simple

local_clean:
	rm -f *.pb.cc *.pb.h *.o

mrproper: local_clean
	rm -f tsp_simple
