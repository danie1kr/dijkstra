# dijkstra
Header-only C++ Dijkstra implementation
[![Compile & Test](https://github.com/danie1kr/dijkstra/actions/workflows/main.yml/badge.svg)](https://github.com/danie1kr/dijkstra/actions/workflows/main.yml)

# About
This repository contains a single C++ header file `dijkstra.hpp` providing a slim interface.
I created this for my other [repository](https://github.com/danie1kr/timeSaverSolver) which can genreate model railroad timesaver puzzles and I wanted it to also provide a best answer ;)

# Requirements
C++14, tested with Visual Studio 2022
(As it needs to run on a microcontroller which compiler does not support the most recent C++ standards, I cannot use the newest features.)

# Usage
The Dijkstra template takes a `_Distance` type and optional `_Tidx` type, defaulting to `size_t`.

```C++
class Step;
using Steps = std::vector<Step>;
using Dijk = Dijkstra<_Distance = unsigned long>;
```

The constructor takes an `infinite` and `unset` property, the later defaulting to `-1`. Both are used for settingthe internal data structures and need to match the `_Distance` and `_Tidx` types.
```C++
Dijk dijkstra(-1);
```

The `dijkstra` method takes a collection size and the index to the first node to start from. Internally, all mechanisms are handled on a index basis. So remember to keep your collection stable between the `dijkstra` call and the `shortestPath` calculation!

Additionally, two callback methods need to be provided:
```C++
std::function<const std::vector<size_t>(const size_t)> neighbors;
std::function<const _Distance(const size_t, const size_t, PreviousCallback)> distance;
```
`neighbors` provides a list of connected nodes to the algorithm and `distance` calculates the length of an edge between two nodes. To better calculate the distance, the whole path can be retrieved via the `PreviousCallback`

```C++
// calculate all paths
Dijk dijkstra;
dijkstra.dijkstra(this->steps.size(), 0,
	[&](const size_t i) -> const std::vector<size_t> {
		std::vector<size_t> neighbors;
		for (auto neighbor : steps[i].actions)
			neighbors.push_back(neighbor.target);
		return neighbors;
	},
	[&](const size_t a, const size_t b, PreviousCallback prec) -> const unsigned int {
		for (auto neighbor : steps[a].actions)
			if (neighbor.target == b)
				return (unsigned int)1;

		return 0;
	}
);
```

A call to `shortestPath` with a index from the `Nodes` collection will then calculate the shortes path, returning a vector of indices:
```C++
// get shortest path to target
using Path = std::vector<const size_t>;
Path path = dijkstra.shortestPath(&target);
```

## One-step calls
To execute the time consuming dijkstra alogrithm on large graphs also in an event based environment without blocking, two indiviual methods are provided:
```C++
		// init
		dijkstra_init(nodesCount, start);

		// dijkstra
		while (dijkstra_step(neighbors, distance));
		
```
This allows the execution on systems with a watchdog, e.g. a browser. It is equivalent to the `dijkstra.dijkstra(nodes, start, neighbors, distance)` call.

Also see [example.cpp](example.cpp) which is used for the status badge.

Let me know of any comments, remarks or additions.
