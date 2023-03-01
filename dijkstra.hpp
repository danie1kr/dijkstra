
#pragma once
#include <vector>
#include <map>
#include <functional>

template<class _Node, class _Nodes, typename _Distance, typename _Distance _Infinity>
class Dijkstra
{
	const _Distance infinity = _Infinity;

	std::vector<const _Node*> Q;
	std::map<const _Node*, _Distance> dist;
	std::map<const _Node*, const _Node*> prec;

public:
	using Path = std::vector<const _Node*>;
	const Path shortestPath(const _Node* target)
	{
		Path path;
		path.push_back(target);
		const _Node* current = target;
		while (prec[current] != nullptr)
		{
			current = prec[current];
			path.push_back(current);
		}

		return path;
	}

	void dijkstra(const _Nodes& nodes, const _Node* start, std::function<const std::vector<const _Node*>(const _Node*)> neighbors, std::function<const _Distance(const _Node*, const _Node*)> distance)
	{
		dist.clear();
		prec.clear();
		Q.clear();

		// init
		for (auto it = nodes.begin(); it != nodes.end(); ++it)
		{
			dist.emplace(&*it, infinity);
			prec.emplace(&*it, nullptr);
			Q.push_back(&*it);
		}
		dist[start] = 0;

		// dijkstra
		while (Q.size() > 0)
		{
			const _Node* u;
			_Distance shortestDistInQ = _Infinity;

			// find Q with shortest distance
			for (auto it = Q.begin(); it != Q.end(); ++it)
			{
				if (dist[*it] < shortestDistInQ)
				{
					u = *it;
					shortestDistInQ = dist[*it];
				}
			}

			// remove it from Q
			(void)std::remove(Q.begin(), Q.end(), u);
			Q.resize(Q.size() - 1);

			// check all neighbors and remember the shortest path
			for (const auto v : neighbors(u))
			{
				if (std::find(Q.begin(), Q.end(), v) != Q.end())
				{
					_Distance dist_u_v = dist[u] + distance(u, v);
					if (dist[u] + dist_u_v < dist[v])
					{
						dist[v] = dist[u] + dist_u_v;
						prec[v] = u;
					}
				}
			}
		}
	}
};