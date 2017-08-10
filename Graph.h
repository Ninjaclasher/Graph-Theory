#ifndef GRAPH_H
#define GRAPH_H

#include <algorithm>
#include <array>
#include <climits>
#include <queue>
#include <stack>
#include <utility>
#include <vector>

using std::greater;
using std::make_pair;
using std::min;
using std::pair;
using std::priority_queue;
using std::queue;
using std::stack;
using std::vector;

class disjoint_set
{
public:
    disjoint_set(int);
    int Find(int);
    void Merge(int,int);

private:
    vector<int> rnk, parent;
    int V;
};

class Graph
{
public:
    Graph(int);
    vector<int> graphBFS(int);
    vector<vector<int>> gridBFS(const vector<vector<char>> &, const char, const int, const int);
    vector<vector<int>> gridBFS(const vector<vector<char>> &, const char, const vector<pair<int,int>> &, const int, const int);
    void MST();
    vector<vector<int>> getCyclesRecursive();
    vector<vector<int>> getCyclesIterative();
    vector<pair<int,int>> getBridges();
    vector<int> getArticulationPoints();
    void reset();
    void resetAll();

    inline void uAddEdge (const int node1, const int node2, const int weight)
    {
        adj[node1].push_back(make_pair(node2, weight));
        adj[node2].push_back(make_pair(node1, weight));
    }
    inline void uAddEdge (const int node1, const int node2)
    {
        adj[node1].push_back(make_pair(node2, 1));
        adj[node2].push_back(make_pair(node1, 1));
    }
    inline void dAddEdge (const int node1, const int node2, const int weight)
        { adj[node1].push_back(make_pair(node2, weight));}
    inline void dAddEdge (const int node1, const int node2)
        { adj[node1].push_back(make_pair(node2, 1));}
    inline void mAddEdge (const int node1, const int node2, const int weight)
        { unprocessedEdges.push_back(make_pair(weight, make_pair(node1, node2)));}

private:
    int V;
    vector<vector<pair<int,int>>> adj;
    vector<pair<int,pair<int,int>>> unprocessedEdges;
    vector<vector<int>> storedCycles;
    vector<int> dist;
    vector<bool> vis;
    void rCycleUtil(int, vector<int> &);
    void bridgeUtil(int, vector<pair<int,int>> &, vector<int> &, vector<pair<int,int>> &);
    void APUtil(int, vector<pair<int,int>> &, vector<int> &, vector<int> &);
};

#endif
