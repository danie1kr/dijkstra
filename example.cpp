#include <string>
#include <vector>
#include <numeric>

#include "dijkstra.hpp"

int main(int argc, const char* const argv[])
{
    using Dijk = Dijkstra<unsigned long>;
	Dijk dijkstra(-1);

    std::vector<std::string> cities = { 
        /* 0: */ "Frankfrut",
        /* 1: */ "Mannheim",
        /* 2: */ "Würzburg",
        /* 3: */ "Stuttgart",
        /* 4: */ "Kassel",
        /* 5: */ "Karlsruhe",
        /* 6: */ "Erfurt",
        /* 7: */ "Nürnberg",
        /* 8: */ "Augsburg",
        /* 9: */ "München"
    };

    std::vector<std::vector<std::pair<size_t, unsigned long>>> connections = {
        /* "Frankfrut" */ { {1, 85}, {2, 217}, {4, 173} },
        /* "Mannheim" */ { {0, 85}, {5, 80} },
        /* "Würzburg" */ { {0, 217}, {6, 186}, {7, 103} },
        /* "Stuttgart" */ { {7, 183} },
        /* "Kassel" */{ {0, 173}, {9, 502} },
        /* "Karlsruhe" */{ {1, 80}, {8, 250} },
        /* "Erfurt" */{ {2, 186} },
        /* "Nürnberg" */{ {2, 103}, {3, 183}, {9, 167} },
        /* "Augsburg" */{ {5, 250 }, {9, 84} },
        /* "München" */{ {8, 84}, {4, 502}, {7, 167} }
    };

    auto distance = [&](const size_t a, const size_t b, Dijk::PreviousCallback prec) -> const unsigned long {
        for (auto neighbor : connections[a])
            if (neighbor.first == b)
                return neighbor.second;

        return 0;
    };

    dijkstra.dijkstra(cities.size(), 0, [&](const size_t i) -> const std::vector<size_t> {
        std::vector<size_t> neighbors;
        for (auto neighbor : connections[i])
            neighbors.push_back(neighbor.first);
        return neighbors;
        },
        distance);

    auto shortestPathDistance = dijkstra.shortestDistance(9, distance);

    return shortestPathDistance == 487 ? 0 : -1;
}