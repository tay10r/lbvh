# This is an optional file that may be
# created by the user to control various build
# parameters.
-include config.mk

PERF = perf

# Setup compiler flags.

CXXFLAGS := -Wall -Wextra -Werror -Wfatal-errors
CXXFLAGS := $(CXXFLAGS) -std=c++17
CXXFLAGS := $(CXXFLAGS) -I .

clang++_CXXFLAGS := -Weverything -Wno-c++98-compat

ifdef LBVH_DEBUG
CXXFLAGS := $(CXXFLAGS) -ggdb
endif

ifdef LBVH_RELEASE
CXXFLAGS := $(CXXFLAGS) -O3 -march=native -ffast-math
endif

ifdef LBVH_NO_THREADS
CXXFLAGS := $(CXXFLAGS) -DLBVH_NO_THREADS=1
else
LDLIBS += -ltbb -lpthread
endif

# Define main programs

test_model := models/sponza.obj

examples += examples/minimal

# Default build target

.PHONY: all
all: lbvh_test $(examples)

# Test program

lbvh_test: lbvh_test.o \
           third-party/stb_image_write.o

lbvh_test.o: lbvh_test.cpp                 \
             lbvh.h                        \
             third-party/stb_image_write.h

# Examples

examples/minimal: examples/minimal.o

examples/minimal.o: examples/minimal.cpp lbvh.h

# Tools

tools/simplify_model: tools/simplify_model.o third-party/tiny_obj_loader.o

tools/simplify_model.o: tools/simplify_model.cpp lbvh.h third-party/tiny_obj_loader.h

# Third party sources

third-party/tiny_obj_loader.o: third-party/tiny_obj_loader.cc \
                               third-party/tiny_obj_loader.h

third-party/stb_image_write.o: third-party/stb_image_write.c \
                               third-party/stb_image_write.h

# Generic patterns

%: %.o
	echo "LINK $@"
	$(CXX) $^ -o $@ $(LDLIBS)

%.o: %.cpp
	echo "CXX  $@"
	$(CXX) $(CXXFLAGS) $($(CXX)_CXXFLAGS) -c $< -o $@

%.o: %.cc
	echo "CXX  $@"
	$(CXX) $(CXXFLAGS) $($(CXX)_CXXFLAGS) -c $< -o $@

%.o: %.c
	echo "CC   $@"
	$(CC) $(CFLAGS) $($(CC)_CFLAGS) -c $< -o $@

# Simplified models

models += simplified-model-float.bin
models += simplified-model-double.bin

$(models): $(test_model) tools/simplify_model
	./tools/simplify_model $(test_model)

# Special targets

.PHONY: clean
clean:
	$(RM) lbvh_test $(examples) $(tools)
	$(RM) *.o *.png *.bin third-party/*.o tools/*.o examples/*.o

.PHONY: test
test: lbvh_test                  \
      simplified-model-float.bin \
      simplified-model-double.bin
	./$<

.PHONY: profile_build
profile_build: lbvh_test                  \
               simplified-model-float.bin \
               simplified-model-double.bin
	$(PERF) record -e cpu-clock,faults,branches,cache-misses --freq=10000 ./lbvh_test --skip-rendering

$(V).SILENT:
