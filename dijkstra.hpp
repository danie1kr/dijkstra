// From: https://github.com/danie1kr/dijkstra
// License: https://github.com/danie1kr/dijkstra/blob/main/LICENSE

#pragma once
#include <vector>
#include <map>
#include <functional>

template<typename _Distance>
class Dijkstra
{
	std::vector<bool> Q;
	size_t remainingQ = 0;

public:
	const _Distance infinity;
	static const size_t unset = (size_t)-1;

	template<typename T>
	struct Storage
	{
		using AllocCallback = std::function<void(const size_t elements, const T defaultValue)>;
		using SetCallback = std::function<void(const size_t i, const T value)>;
		using GetCallback = std::function<const T(const size_t i)>;

		using StorageType = T;

		AllocCallback alloc;
		SetCallback set;
		GetCallback get;

		Storage() = delete;
		Storage(Storage&) = delete;
		Storage(Storage&&) = delete;
		Storage(AllocCallback alloc, SetCallback set, GetCallback get) :
			alloc{ alloc }, set{ set }, get{ get }
		{

		};
	};

	using DistanceStorage = Storage<_Distance>;
	using PrecStorage = Storage<size_t>;

private:
	DistanceStorage& dist;
	PrecStorage& prec;

public:
	using Path = std::vector<size_t>;
	const Path shortestPath(const size_t target) const
	{
		Path path;
		path.push_back(target);
		size_t current = target;
		while (prec.get(current) != unset)
		{
			current = prec.get(current);
			path.push_back(current);
		}

		return path;
	}

	const _Distance shortestDistance(const size_t target, std::function<const _Distance(const size_t, const size_t, typename PrecStorage::GetCallback)> distance) const
	{
		_Distance dist = 0;
		size_t current = target, next = target;
		while ((next = prec.get(current)) != unset)
		{
			dist += distance(current, next, prec.get);
			current = next;
		}

		return dist;
	}

	using PathCallback = std::function<void(const size_t i)>;
	void shortestPath(const size_t target, PathCallback path) const
	{
		size_t current = target;
		size_t j = unset;
		while ((j = prec.get(current)) != unset)
			path(j);
	}

	Dijkstra(const _Distance infinity, DistanceStorage& dist, PrecStorage& prec) : infinity{ infinity }, dist{ dist }, prec{ prec } { };

	void dijkstra_init(const size_t nodesCount, const size_t start)
	{
		const size_t n = nodesCount;
		dist.alloc(n, infinity);
		prec.alloc(n, unset);
		Q.resize(n, false);
		
		// init
		for (size_t i = 0; i < n; ++i)
		{
			dist.set(i, infinity);
		//	prec.set(i, unset);
		//	Q[i] = false;
		}
		dist.set(start, 0);
		remainingQ = n;
	}

	const bool dijkstra_step(std::function<const std::vector<size_t>(const size_t)> neighbors, std::function<const _Distance(const size_t, const size_t, typename PrecStorage::GetCallback )> distance)
	{
		if (remainingQ > 0)
		{
			size_t u = unset;
			_Distance shortestDistInQ = this->infinity;

			// find Q with shortest distance
			for (auto i = 0; i < Q.size(); ++i)
			{
				if (!Q[i])
				{
					const auto d = dist.get(i);
					if (d < shortestDistInQ)
					{
						u = i;
						shortestDistInQ = d;
					}
				}
			}

			// we found another independent cycle in the graph
			if (u == unset)
			{
				return false;
			}

			// remove it from Q
			Q[u] = true;
			--remainingQ;

			// check all neighbors and remember the shortest path
			for (const auto v : neighbors(u))
			{
				if (Q[v] == false)
				{
					const auto dist_u = dist.get(u);
					const auto dist_v = dist.get(v);
					const auto dist_u_v = distance(u, v, prec.get);
					if (dist_u + dist_u_v < dist_v)
					{
						dist.set(v, dist_u + dist_u_v);
						prec.set(v, u);
					}
				}
			}
		}
		return remainingQ > 0;
	}

	// convenience wrapper for the three methods
	void dijkstra(const size_t nodesCount, const size_t start, std::function<const std::vector<size_t>(const size_t)> neighbors, std::function<const _Distance(const size_t, const size_t, typename PrecStorage::GetCallback)> distance)
	{
		this->dijkstra_init(nodesCount, start);

		// dijkstra
		while (this->dijkstra_step(neighbors, distance))
			;
	}
};

#include <queue>
template<typename _Distance>
class DijkstraInternalMemory
{
	std::vector<bool> Q;
	std::vector<_Distance> dist;
	std::vector<size_t> prec;
	size_t remainingQ = 0;

public:
	const _Distance infinity;
	//static const size_t unset = (size_t)-1;

	template<typename T>
	struct Storage
	{
		using GetCallback = std::function<const T(const size_t i)>;
	};

	using PrecStorage = Storage<size_t>;

public:
	using Path = std::vector<size_t>;
	const Path shortestPath(const size_t target)
	{
		Path path;
		path.push_back(target);
		size_t current = target;
		while (prec[current] != (size_t)-1)
		{
			current = prec[current];
			path.push_back(current);
		}

		return path;
	}

	using PathCallback = std::function<void(const size_t i)>;
	void shortestPath(const size_t target, PathCallback path)
	{
		size_t current = target;
		size_t j = (size_t)-1;
		while ((j = prec[current]) != (size_t)-1)
			path(j);
	}

	DijkstraInternalMemory(const _Distance infinity) : infinity{ infinity }, dist(), prec() { };
	
	struct DistNodePair
	{
		DistNodePair(const DistNodePair &other) : dist{ other.dist }, node{ other.node } {};
		DistNodePair(const _Distance dist, const size_t node) : dist{ dist }, node{ node } {};
		DistNodePair& operator=(DistNodePair other)
		{
			std::swap(dist, other.dist);
			std::swap(node, other.node);
			return *this;
		}

		_Distance dist;
		size_t node;
		bool operator>(const DistNodePair& b) const
		{
			return this->dist > b.dist;
		}
	};
	std::priority_queue<DistNodePair, std::vector<DistNodePair>, std::greater<DistNodePair>> pq;

	void dijkstra_init(const size_t nodesCount, const size_t start)
	{
		const size_t n = nodesCount;
		dist.resize(n, infinity);
		prec.resize(n, (size_t)-1);
		pq.emplace(0, start);
		dist[start] = 0;

		/*
		const size_t n = nodesCount;
		dist.resize(n, infinity);
		prec.resize(n, unset);
		Q.resize(n, false);

		dist[start] = 0;
		Q[start] = true;
		remainingQ = n - 1;*/
	}

	const bool dijkstra_step(std::function<const std::vector<size_t>(const size_t)> neighbors, std::function<const _Distance(const size_t, const size_t, typename PrecStorage::GetCallback)> distance)
	{
		if (!pq.empty())
		{
			const auto u = pq.top().node;
			pq.pop();
			// check all neighbors and remember the shortest path
			for (const auto v : neighbors(u))
			{
				//if (Q[v] == false)
				{
					const auto dist_u = dist[u];
					const auto dist_v = dist[v];
					const auto dist_u_v = distance(u, v, [this](const size_t i) -> const size_t { return prec[i]; });
					if (dist_u + dist_u_v < dist_v)
					{
						dist[v] = dist_u + dist_u_v;
						prec[v] = u;
						pq.emplace(dist[v], v);
					}
				}
			}

		}
		return pq.size() > 0;
		/*
		if (remainingQ > 0)
		{
			size_t u = unset;
			_Distance shortestDistInQ = this->infinity;

			// find Q with shortest distance
			for (auto i = 0; i < Q.size(); ++i)
			{
				if (!Q[i])
				{
					const auto d = dist[i];
					if (d < shortestDistInQ)
					{
						u = i;
						shortestDistInQ = d;
					}
				}
			}

			// we found another independent cycle in the graph
			if (u == unset)
			{
				return false;
			}

			// remove it from Q
			Q[u] = true;
			--remainingQ;

			// check all neighbors and remember the shortest path
			for (const auto v : neighbors(u))
			{
				if (Q[v] == false)
				{
					const auto dist_u = dist[u];
					const auto dist_v = dist[v];
					const auto dist_u_v = distance(u, v, [this](const size_t i) -> const size_t { return prec[i]; });
					if (dist_u + dist_u_v < dist_v)
					{
						dist[v] = dist_u + dist_u_v;
						prec[v] = u;
					}
				}
			}
		}
		return remainingQ > 0;*/
	}

	// convenience wrapper for the three methods
	void dijkstra(const size_t nodesCount, const size_t start, std::function<const std::vector<size_t>(const size_t)> neighbors, std::function<const _Distance(const size_t, const size_t, typename PrecStorage::GetCallback)> distance)
	{
		this->dijkstra_init(nodesCount, start);

		// dijkstra
		while (this->dijkstra_step(neighbors, distance))
			;
	}
};