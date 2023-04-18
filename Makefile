OR_TOOLS_TOP=../or-tools

TUTORIAL=./resources

# -isystem prevents most of the warnings rooted in or-tools library appearing in our compilation
CFLAGS := -std=c++14 -isystem$(OR_TOOLS_TOP)/include

# During development uncomment the next line to have debug checks and other verifications
# DEVELOPMENT = true
ifeq ($(DEVELOPMENT), true)
  CFLAGS := $(CFLAGS) -O0 -DDEBUG -ggdb3 -fsanitize=address -fkeep-inline-functions -fno-inline-small-functions
	# CXX := ../callcatcher/build/scripts-3.6/callcatcher $(CXX) # comment out to check uncalled functions
  CXX := LSAN_OPTION=verbosity=1:log_threads=1 $(CXX) # adress sanitizer works only if the executable launched without gdb
else
  CFLAGS := $(CFLAGS) -O3 -DNDEBUG
endif

# Activate warnings
CFLAGS := $(CFLAGS) -Wall -Wextra -Wshadow -Wmissing-include-dirs -Wswitch-enum -Wfloat-equal -Wundef

# The following is to supress a warning due to a protobuf that is fixed at v3.14.
# It can be removed when or-tools is upgraded to v8.1+ (where the protobuf dependency is upgraded to v3.14).
PROTOBUF_VERSION := $(shell $(OR_TOOLS_TOP)/bin/protoc --version | cut -d" " -f2)
ifeq ($(shell dpkg --compare-versions $(PROTOBUF_VERSION) 'lt' '3.14' && echo true), true)
	CFLAGS := $(CFLAGS) -Wno-array-bounds
endif

.PHONY: all local_clean

all: $(EXE)

%.pb.cc: %.proto
	$(OR_TOOLS_TOP)/bin/protoc --cpp_out . --ruby_out . $<

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
	$(CXX) $(CFLAGS) -isystem$(TUTORIAL) -c ./tsp_simple.cc -o tsp_simple.o

tsp_simple: $(ROUTING_DEPS) tsp_simple.o ortools_vrp.pb.o ortools_result.pb.o $(OR_TOOLS_TOP)/lib/libortools.so
	$(CXX) $(CFLAGS) -fwhole-program tsp_simple.o ortools_vrp.pb.o ortools_result.pb.o $(OR_TOOLS_LD_FLAGS) \
	-L $(OR_TOOLS_TOP)/lib -Wl,-rpath $(OR_TOOLS_TOP)/lib -lortools -lprotobuf -lglog -lgflags -labsl_raw_hash_set -labsl_time -labsl_time_zone \
	-o tsp_simple

local_clean:
	rm -f *.pb.cc *.pb.h *.o

mrproper: local_clean
	rm -f tsp_simple
