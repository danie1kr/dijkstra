// From: https://github.com/danie1kr/dijkstra
// License: https://github.com/danie1kr/dijkstra/blob/main/LICENSE

#pragma once
#include <vector>
#include <functional>
#include <queue>

template<typename _Distance, typename _Tidx = size_t>
class Dijkstra
{
	std::vector<_Distance> dist;
	std::vector<_Tidx> prec;

public:
	const _Distance infinity;
	static const _Tidx unset = (_Tidx)-1;

	using PreviousCallback = std::function<const _Tidx(const _Tidx i)>;

public:
	using Path = std::vector<_Tidx>;
	const Path shortestPath(const _Tidx target)
	{
		Path path;
		path.push_back(target);
		_Tidx current = target;
		while (prec[current] != unset)
		{
			current = prec[current];
			path.push_back(current);
		}

		return path;
	}

	using PathCallback = std::function<void(const _Tidx i)>;
	void shortestPath(const _Tidx target, PathCallback path)
	{
		_Tidx current = target;
		_Tidx j = unset;
		while ((j = prec[current]) != unset)
			path(j);
	}

	const _Distance shortestDistance(const _Tidx target, std::function<const _Distance(const _Tidx, const _Tidx, typename PreviousCallback)> distance) const
	{
		_Distance dist = 0;
		_Tidx current = target, next = target;
		while ((next = prec[current]) != unset)
		{
			dist += distance(current, next, [this](const _Tidx i) -> const _Tidx { return prec[i]; });
			current = next;
		}

		return dist;
	}


	Dijkstra(const _Distance infinity) : infinity{ infinity } { };
	
	struct DistNodePair
	{
		DistNodePair(const DistNodePair &other) : dist{ other.dist }, node{ other.node } {};
		DistNodePair(const _Distance dist, const _Tidx node) : dist{ dist }, node{ node } {};
		DistNodePair& operator=(DistNodePair other)
		{
			std::swap(dist, other.dist);
			std::swap(node, other.node);
			return *this;
		}

		_Distance dist;
		_Tidx node;
		bool operator>(const DistNodePair& b) const
		{
			return this->dist > b.dist;
		}
	};
	std::priority_queue<DistNodePair, std::vector<DistNodePair>, std::greater<DistNodePair>> pq;

	void dijkstra_init(const _Tidx nodesCount, const _Tidx start)
	{
		const _Tidx n = nodesCount;
		dist.resize(n, infinity);
		prec.resize(n, unset);
		pq.emplace(0, start);
		dist[start] = 0;
	}

	// see also https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
	const bool dijkstra_step(std::function<const std::vector<_Tidx>(const _Tidx)> neighbors, std::function<const _Distance(const _Tidx, const _Tidx, typename PreviousCallback)> distance)
	{
		if (!pq.empty())
		{
			const auto u = pq.top().node;
			pq.pop();
			// check all neighbors and remember the shortest path
			for (const auto v : neighbors(u))
			{
				const auto dist_u = dist[u];
				const auto dist_v = dist[v];
				const auto dist_u_v = distance(u, v, [this](const _Tidx i) -> const _Tidx { return prec[i]; });
				if (dist_u + dist_u_v < dist_v)
				{
					dist[v] = dist_u + dist_u_v;
					prec[v] = u;
					pq.emplace(dist[v], v);
				}
			}

		}
		return pq.size() > 0;
	}

	// convenience wrapper for the three methods
	void dijkstra(const _Tidx nodesCount, const _Tidx start, std::function<const std::vector<_Tidx>(const _Tidx)> neighbors, std::function<const _Distance(const _Tidx, const _Tidx, typename PreviousCallback)> distance)
	{
		this->dijkstra_init(nodesCount, start);

		// dijkstra
		while (this->dijkstra_step(neighbors, distance))
			;
	}
};