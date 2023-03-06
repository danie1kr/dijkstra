# dijkstra
Header-only C++ Dijkstra implementation

# About
This repository contains a single C++ header file `dijkstra.hpp` providing a slim but highly flexible interface.
I created this for my other [repository](https://github.com/danie1kr/timeSaverSolver) which can genreate model railroad timesaver puzzles and I wanted it to also provide a best answer ;)

# Requirements
C++14, tested with Visual Studio 2022
(As it needs to run on a microcontroller which compiler does not support the most recent C++ standards, I cannot use the newest features.)

# Usage
The Dijkstra template takes a `_Node` class, a `_Nodes` collection, a `_Distance` type and a value for `_Infinity`.

```C++
class Step;
using Steps = std::vector<Step>;
using Dijk = Dijkstra<_Node = Step, _Nodes = Steps, _Distance = unsigned int, _Infinity = 0xFFFFFFFF>;
```

The `dijkstra` method takes a `Nodes` collection and the pointer to the first node to start from. Internally, all mechanisms are handled on a pointer basis. So remember to keep your collection stable between the `dijkstra` call and the `shortestPath` calculation!

Additionally, two callback methods need to be provided:
```C++
std::function<const std::vector<const _Node*>(const _Node*)> neighbors;
std::function<const _Distance(const _Node*, const _Node*)> distance;
```
`neighbors` provides a list of connected nodes to the algorithm and `distance` calculates the length of an edge between two nodes.

```C++
// calculate all paths
Dijk dijkstra;
dijkstra.dijkstra(this->steps, &this->steps[0],
	[&](const Step* node) -> const std::vector<const Step*> {
		std::vector<const Step*> neighbors;
		for (auto neighbor: node->actions)
			neighbors.push_back(&steps[neighbor.target]);
		return neighbors;
	},
	[](const Step* a, const Step* b) -> const unsigned int {
		for (auto neighbor : a->actions)
			if (neighbor.target == b->id)
				return 1;
		return 0xFFFFFFFF;
	});
```

A call to `shortestPath` with a pointer from the `Nodes` collection will then calculate the shortes path, returning a vector of nodes:
```C++
// get shortest path to target
using Path = std::vector<const _Node*>;
Path path = dijkstra.shortestPath(&target);
```

## One-step calls
To execute the time consuming dijkstra alogrithm on large graphs also in an event based environment without blocking, two indiviual methods are provided:
```C++
		// init
		dijkstra_init(nodes, start);

		// dijkstra
		while (dijkstra_step(neighbors, distance));
		
```
This allows the execution on systems with a watchdog, e.g. a browser. It is equivalent to the `dijkstra.dijkstra(nodes, start, neighbors, distance)` call.

Let me know of any comments, remarks or additions.