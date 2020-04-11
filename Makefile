-include config.mk

CXXFLAGS := -Wall -Wextra -Werror -Wfatal-errors
CXXFLAGS := $(CXXFLAGS) -std=c++17
CXXFLAGS := $(CXXFLAGS) -I .

clang++_CXXFLAGS := -Weverything -Wno-c++98-compat

ifndef LBVH_DEBUG
CXXFLAGS := $(CXXFLAGS) -O3 -march=native -ffast-math
else
CXXFLAGS := $(CXXFLAGS) -g
endif

ifdef LBVH_NO_THREADS
CXXFLAGS := $(CXXFLAGS) -DLBVH_NO_THREADS=1
else
LDLIBS += -ltbb -lpthread
endif

examples += examples/minimal

.PHONY: all
all: lbvh_test $(examples)

.PHONY: test
test: lbvh_test
	./$< --errors_fatal ./models/sponza.obj

examples/minimal: examples/minimal.cpp lbvh.h

examples/%: examples/%.cpp lbvh.h
	$(CXX) $(CXXFLAGS) $($(CXX)_CXXFLAGS) $< -o $@ $(LDLIBS)

lbvh_test: lbvh_test.o models/tiny_obj_loader.o
	$(CXX) $^ -o $@ $(LDLIBS)

lbvh_test.o: lbvh_test.cpp lbvh.h models/model.h models/tiny_obj_loader.h
	$(CXX) $(CXXFLAGS) $($(CXX)_CXXFLAGS) -c $< -o $@

models/tiny_obj_loader.o: models/tiny_obj_loader.cc models/tiny_obj_loader.h
	$(CXX) $(CXXFLAGS) $($(CXX)_CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(RM) lbvh_test $(examples) *.o models/*.o
