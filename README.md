# dijkstra
Header-only C++ Dijkstra implementation

# About
This repository contains a single C++ header file `dijkstra.hpp` providing a slim but highly flexible interface.
I created this for my other [repository](https://github.com/danie1kr/timeSaverSolver) which can genreate model railroad timesaver puzzles and I wanted it to also provide a best answer ;)

# Requirements
C++14, tested with Visual Studio 2022

# Usage

The Dijkstra template takes a `Node` class, a `Nodes` collection, a `distance` type and a value for `infinity`.

```C++
class Step;
using Steps = std::vector<Step>;
using Dijk = Dijkstra<Step, Steps, unsigned int, 0xFFFFFFFF>;
```

The `dijkstra` method takes a `Nodes` collection and the pointer to the first node to start from. Internally, all mechanisms are handled on a pointer basis.

Additionally, two callback methods need to be provided:
```C++
std::function<const std::vector<const _Node*>(const _Node*)> neighbors;
std::function<const _Distance(const _Node*, const _Node*)> distance;
```
`neighbors` provides a list of connected nodes to the algorithm and `distance` calculates the length of an edge between two nodes.

```C++
// get shortest path
Dijk dijkstra;
dijkstra.dijkstra(this->steps, &this->steps[0],
	[&](const Step* node) -> const std::vector<const Step*> {
		std::vector<const Step*> neighbors;
		for (auto neighbor : node->actions)
			neighbors.push_back(&steps[neighbor.target]);
		return neighbors;
	},
	[](const Step* a, const Step* b) -> const unsigned int {
		for (auto neighbor : a->actions)
			if (neighbor.target == b->id)
				return (unsigned int)1;

		return 0;
	});
```

A call to `shortestPath` with a pointer from the `Nodes` collection will then calculate the shortes path, returning a vector of nodes:
```C++
using Path = std::vector<const _Node*>;
Path path = dijkstra.shortestPath(&step);
```