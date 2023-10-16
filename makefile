CXX = g++
CXXFLAGS = -std=c++11

.PHONY: clean test

test: p1
	./p1 test.in out.txt

p1: p1.cpp
	$(CXX) $(CXXFLAGS) -o p1 p1.cpp

clean:
	-rm -f p1 out.txt
