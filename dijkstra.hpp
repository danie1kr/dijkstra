// From: https://github.com/danie1kr/dijkstra
// License: https://github.com/danie1kr/dijkstra/blob/main/LICENSE

#pragma once
#include <vector>
#include <map>
#include <functional>

template<typename _Distance>
class Dijkstra
{
	//std::vector<const _Node*> Q;
	//std::map<const _Node*, _Distance> dist;
	//std::map<const _Node*, const _Node*> prec;

	std::vector<bool> Q;
	size_t remainingQ = 0;

	const size_t unset = (size_t)-1;

public:
	const _Distance infinity;

	template<typename T>
	struct Storage
	{
		using AllocCallback = std::function<void(const size_t elements, const size_t sizePerElement)>;
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
	const Path shortestPath(const size_t target)
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

	using PathCallback = std::function<void(const size_t i)>;
	void shortestPath(const size_t target, PathCallback path)
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
		dist.alloc(n, sizeof(_Distance));
		prec.alloc(n, sizeof(size_t));
		Q.resize(n);

		// init
		for (size_t i = 0; i < n; ++i)
		{
			dist.set(i, infinity);
			prec.set(i, unset);
			Q[i] = false;
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

/*
template<class _Node, class _Nodes, typename _Distance>
class Dijkstra
{
	std::vector<const _Node*> Q;
	std::map<const _Node*, _Distance> dist;
	std::map<const _Node*, const _Node*> prec;

public:
	const _Distance infinity;

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

	Dijkstra(const _Distance infinity) : infinity(infinity) { };

	void dijkstra_init(const _Nodes& nodes, const _Node* start)
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
	}

	const bool dijkstra_step(std::function<const std::vector<const _Node*>(const _Node*)> neighbors, std::function<const _Distance(const _Node*, const _Node*, const std::map<const _Node*, const _Node*>&)> distance)
	{
		if (Q.size() > 0)
		{
			const _Node* u = nullptr;
			_Distance shortestDistInQ = this->infinity;

			// find Q with shortest distance
			for (auto it = Q.begin(); it != Q.end(); ++it)
			{
				if (dist[*it] < shortestDistInQ)
				{
					u = *it;
					shortestDistInQ = dist[*it];
				}
			}

			// we found another independent cycle in the graph
			if (u == nullptr)
			{
				return false;
			}

			// remove it from Q
			(void)std::remove(Q.begin(), Q.end(), u);
			Q.resize(Q.size() - 1);
			 
			// check all neighbors and remember the shortest path
			for (const auto v : neighbors(u))
			{
				if (std::find(Q.begin(), Q.end(), v) != Q.end())
				{
					_Distance dist_u_v = distance(u, v, prec);
					if (dist[u] + dist_u_v < dist[v])
					{
						dist[v] = dist[u] + dist_u_v;
						prec[v] = u;
					}
				}
			}
		}
		return Q.size() > 0;
	}

	// convenience wrapper for the three methods
	void dijkstra(const _Nodes& nodes, const _Node* start, std::function<const std::vector<const _Node*>(const _Node*)> neighbors, std::function<const _Distance(const _Node*, const _Node*, const std::map<const _Node*, const _Node*>&)> distance)
	{
		this->dijkstra_init(nodes, start);

		// dijkstra
		while (this->dijkstra_step(neighbors, distance))
			;
	}
};
*/