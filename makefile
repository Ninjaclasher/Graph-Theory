graph: a.out
	./a.out
a.out: main.cpp Graph.cpp Graph.h Graph.h.gch
	g++ main.cpp Graph.cpp
Graph.h.gch: Graph.h
	g++ Graph.h