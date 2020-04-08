-include config.mk

CXXFLAGS := -Wall -Wextra -Werror -Wfatal-errors
CXXFLAGS := $(CXXFLAGS) -std=c++17
CXXFLAGS := $(CXXFLAGS) -I .

ifndef LBVH_DEBUG
CXXFLAGS := $(CXXFLAGS) -O3 -march=native
else
CXXFLAGS := $(CXXFLAGS) -g
endif

ifdef LBVH_NO_THREADS
CXXFLAGS := $(CXXFLAGS) -DLBVH_NO_THREADS=1
else
LDLIBS += -ltbb
endif

examples += examples/minimal

.PHONY: all
all: lbvh_test $(examples)

.PHONY: test
test: lbvh_test
	./$< --errors_fatal ./models/sponza.obj

examples/minimal: examples/minimal.cpp lbvh.h

examples/%: examples/%.cpp lbvh.h
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

lbvh_test: lbvh_test.cpp lbvh.h
	$(CXX) $(CXXFLAGS) $< models/tiny_obj_loader.cc -o $@ $(LDLIBS)

.PHONY: clean
clean:
	$(RM) lbvh_test $(examples)
