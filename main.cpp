#include<iostream>
#include<vector>
#include<list> // For adjacency list
#include<queue> // For priority queue
#include<limits> // For numeric limits
#include<algorithm> // For reverse

using namespace std;

// Define infinity as a large enough value
const int INF = numeric_limits<int>::max(); 

// Structure to represent a weighted edge in the graph
struct Edge
{
  int to;
  int weight;
};

// Graph class
class Graph
{
  public:
    int numVertices;
    // Adjacency list: vector of lists, where each list contains pairs of (destination_vertex, weight)
    vector<list<Edge>> adj;

    Graph(int V) : numVertices(V), adj(V);

    // Add an edge to a directed graph
    void addEdge(int u, int v, int weight)
    {
      if(u >= 0 && u < numVertices && v >= 0 && v < numVertices)
        {
            adj[u].push_back({v,weight});
           // If it's an undirected graph, add the reverse edge too:
          // adj[v].push_back({u,weight});
        }
      else 
        {
          cerr<<"Error: Vertex out of bounds."<<endl;
        }
    }

    // Dijkstra's algorithm to find the shortest path from a source vertex
    void dijkstraShortestPath(int src, int dest)
    {
      if (src <0 || src >= numVertices || dest < 0 || dest >= numVertices){
        cerr<<"Error: Source or destination vertex out of bounds. "<< endl;
        return;
      }
      // dist[i] will hold the shortest distance from src to i
      vector<int> dist(numVertices, INF);

      // parent[i] will store the predecessor of i in the shortest path from src
      vector<int> parent(numVertices, -1);

      // Priority queue to store vertices that are being preprocessed.
      // We use pair<int, int> where first element is distance and second is vertex label.
      // Greater makes it a min-priority queue (smallest distance on top).
      priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

      // Distance from source to itself is always 0.
      dist[src] = 0;
      pq.push({0, src}); // {distance, vertex}

      while (!pq.empty())
      {
        int u = pq.top().second; // Vertex with the smallest distance
        int d_u = pq.top().first; // Distance to u
        pq.pop();

        // If we found a shorter path already, skip
        // This happens id a vertex is pushed multiple times into PQ with different distances 

        if(d_u > dist[u])
        {
          continue;
        }

        // If we reached the destination, we can potentially stop early
        // For finding all shortest paths from src, remove this check.
        if(u == dest && dist[u] != INF )
        {
          // break; // Optimization: stop if destination is reached
        }

        // Iterate over all adjacent vertices of u
        for (const auto& edge : adj[u]) 
          {
            int v = edge. to;
            int weight_uv = edge. weight;

            // If there is a shorter path to v through u
            if (dist[u] != INF && dist[u] + weight_uv < dist[v]) 
            {
              dist[v] = dist[u] + weight_uv;
              parent[v] = u;
              pq.push({dist[v], v});
            }
          }
      }

      // Print the shortest path and distance
      cout << "Shortest path from vertex " << " to vertex " << dest << ":" << endl;
      if (dist[dest] == INF) 
      {
        cout << "No path exists." << endl;
      }
      else
      {
        cout << "Distance: " << dist[dest] << endl;
        cout << "Path: ";
        vector<int> path;
        int current = dest;
        while (current != -1)
          {
            path.push_back(current);
            current = parent[current];
          }
        reverse(path.begin(), path.end());
        for (size_t i = 0; i < path.size(); ++i)
          {
            cout << path[i] << (i == path. size() - 1? " " : " -> ");
          }
        cout << endl;
      }
    }

    void printGraph() const;
};

int main(){

  return 0;
}
