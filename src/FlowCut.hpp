#ifndef FLOWCUT_HPP_
#define FLOWCUT_HPP_

//
// Dinic's maximum flow
//
// Description:
//   Given a directed network G = (V, E) with edge capacity c: E->R.
//   The algorithm finds a maximum flow.
//
// Algorithm:
//   Dinic's blocking flow algorithm.
//
// Complexity:
//   O(n^2 m), but very fast in practice.
//   In particular, for a unit capacity graph,
//   it runs in O(m min{m^{1/2}, n^{2/3}}).
//
// Verified:
//   SPOJ FASTFLOW
//
// Reference:
//   E. A. Dinic (1970):
//   Algorithm for solution of a problem of maximum flow in networks with power estimation.
//   Soviet Mathematics Doklady, vol. 11, pp. 1277-1280.
//
//   B. H. Korte and J. Vygen (2008):
//   Combinatorial Optimization: Theory and Algorithms.
//   Springer Berlin Heidelberg.
//

#include "FlowCut.hpp"
#include <iostream>
#include <vector>
#include <cstdio>
#include <queue>
#include <algorithm>
#include <functional>

using namespace std;

#define fst first
#define snd second
#define all(c) ((c).begin()), ((c).end())

const long long INF = (1ll << 50); // multiply 1 by 2^50. 1LL is changing 1 from int to long long type

struct FlowCut {
	typedef long long flow_type;
	struct edge {
		int src, dst;
		flow_type capacity, flow;
		size_t rev; // stores the index of the current opposite edge in the dst vector
	};
	int n; //num vertex
	vector<vector<edge>> adj; //adj[x][y]. x is the index of the vertex. y stores edges info that belongs to the indexed (x) vertex
	FlowCut(int n) : n(n), adj(n) { }

	void add_edge(int src, int dst, flow_type capacity) {
		adj[src].push_back({src, dst, capacity, 0, adj[dst].size()});
		adj[dst].push_back({dst, src, 0, 0, adj[src].size() - 1});
	}

	flow_type max_flow(int s, int t) {
		vector<int> level(n), iter(n);

		// function to assign levels in a level graph
		function<int(void)> levelize = [&]() { // foward levelize // & is used to allow variables inside "max_flow" function to be accessed in "levelize"
			level.assign(n, -1); //assign -1 to all vertex levels
			level[s] = 0; //source level = 0
			queue<int> Q; // Q is a first in first out queue
			Q.push(s);// put source into queue
			while (!Q.empty()) {
				int u = Q.front(); //set u to be the vertex at front of queue
				Q.pop(); // remove the vertex at front of queue
				if (u == t) break; // stop the level graph if front of queue is the sink vertex
				for (auto &e : adj[u]) { //for all the edges of the current vertex at the front of the queue;
					if (e.capacity > e.flow && level[e.dst] < 0) { //if edge capacity is larger than flow and level of the destination vertex has not been assigned (not assigned = -1)
						Q.push(e.dst); // add the destination vertex into the queue to be search for branching vertex
						level[e.dst] = level[u] + 1; // give the destination vertex a level
					}
				}
			}
			return level[t]; //returns the level of the sink vertex
		};

		function<flow_type(int, flow_type)> augment = [&](int u, flow_type cur) {
			if (u == t) return cur; //if current vertex is the same as the sink vertex, return INF
			for (int &i = iter[u]; i < adj[u].size(); ++i) { //link "i" to the address of the variable that is calculating num_of_iterations of current vertex. Max "i" can iterate is the number of edges for the "u"th vertex. i is using iter's reference because it wants to reuse the iter count even when called in resursive function
				edge &e = adj[u][i]; // "e" gets the "i"th edge that belongs to "u"th vertex.
				edge &r = adj[e.dst][e.rev]; //"r" gets the edge that stores the residual capacity in the destination vertex of the "i"th edge of the "u"th vertex
				if (e.capacity > e.flow && level[u] < level[e.dst]) { //search through all the vertices of the incremental level and see if their flow is below capacity
					flow_type f = augment(e.dst, min(cur, e.capacity - e.flow));// recursive function will get linked through all the levels to the final vertex (sink) while searching for the min of the (capacity-flow) of all edges. Once final level is reached, then the recursive function will return one by one
					if (f > 0) {
						e.flow += f; //update the flow of ith edge of (uth vertex + level[dst]) to minimum capacity-flow magnitude
						r.flow -= f; //update the residual flow in opposite direction
						return f; // return the magnitude of flow to be updated in all preceding vertex of this recursive function
					}
				}
			}
			return flow_type(0); // return 0 as the flow magnitude to be updated
		};


		for (int u = 0; u < n; ++u) // initialize
			for (auto &e : adj[u]) e.flow = 0; // initialize all edges flow to be 0

		flow_type flow = 0;
		while (levelize() >= 0) { //while the level of the sink is not negative; not negative means there is no edges that can connect from source to sink
			fill(all(iter), 0); // fill iter variable with 0
			for (flow_type f; (f = augment(s, INF)) > 0; ) // update the flow for this iteration, while levelize >= 0; meaning the sink can still be reached in level graph
				flow += f; // sum all the flow magnitude that is being updated
		}
		return flow; // return the max flow value
	}

	vector<int> saturated_vertices() //find vertices with saturated input edges which means they belong to partition T which contains sink. Vertices without saturated edges belong to partition S which contains source
	{
		vector<int> saturated;
		for(int u = 0; u<n; u++) //iterate through all the vertices
		{
			for(edge &e: adj[u]) //iterate through all the edges which belongs to the vertex
			{
				if(e.capacity == e.flow)
				{
					saturated.push_back(e.dst);
					break;
				}
			}
		}
		return saturated;
	}
};



#endif  // FLOWCUT_HPP_