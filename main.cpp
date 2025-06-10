#include<iostream>
#include<vector>
#include<list>
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

    void dijkstraShortestPath(int src, int dest);

    void printGraph() const;
};

int main(){

  return 0;
}
