
#include "FlowCut.hpp"
#include <vector>


int main() {
	int num_vertex, num_edges;
	printf("Number of vertex: ");
	scanf_s("%d", &num_vertex);
	printf("Number of edges: ");
	scanf_s("%d", &num_edges);
	
		FlowCut g(num_vertex);
		for (int i = 0; i < num_edges; ++i) {
			int u, v, w; //u = source, v = destination, w = capacity
			printf("Source, Dst, Capacity: ");
			int numret = scanf("%d %d %d", &u, &v, &w);

			g.add_edge(u, v, w);
		}
		printf("Max flow between vertex %d and %d = %lld\n", 0, num_vertex-1, g.max_flow(0, num_vertex - 1));

	vector<int> saturated_vertices = g.saturated_vertices();
	printf("Saturated Vertices = ");
	for(int i=0; i<saturated_vertices.size(); i++)
	printf("%d\t", saturated_vertices[i]);
}