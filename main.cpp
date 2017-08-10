#include <iostream>
#include <climits>
#include "Graph.h"

using namespace std;

//Driver function to test each utility
int main()
{
    int n, m, a, b, c, src;
    cout<<"Enter number of nodes in the graph"<<endl;
    cin>>n;
    cout<<"Enter number of edges in the graph"<<endl;
    cin>>m;
    Graph g(n+1);
    cout<<"Enter the "<<m<<" connections in the graph in the form (node1 node2 weight)"<<endl;
    for (int x = 0; x < m; x++)
    {
        cin>>a>>b>>c;
        g.uAddEdge(a, b, c);
        g.mAddEdge(a, b, c);
    }

    vector<pair<int,int>> bridges = g.getBridges();
    if (bridges.empty())
        cout<<"There are no bridges in the graph"<<endl;
    else
    {
        cout<<"Bridges in the graph:"<<endl;
        for (auto &x : bridges)
            cout<<"Bridge between "<<x.first<<" and "<<x.second<<endl;
    }
    vector<int> articulationNodes = g.getArticulationPoints();
    if (articulationNodes.empty())
        cout<<"There are no articulation nodes in the graph"<<endl;
    else
    {
        cout<<"Articulation Nodes in the graph:"<<endl;
        for (auto &x : articulationNodes)
            cout<<x<<" is an articulation node in the graph"<<endl;
    }

    cout<<"Getting cycles using iterative function..."<<endl;
    vector<vector<int>> cycles = g.getCyclesIterative();
    if (cycles.empty())
        cout<<"There are no cycles in the graph"<<endl;
    else
    {
        cout<<"Total number of cycles in the graph: "<<cycles.size()<<endl;
        for (auto &x : cycles)
        {
            for (auto &y : x)
                cout<<y;
            cout<<endl;
        }
    }

    cout<<"Getting cycles using recursive function..."<<endl;
    cycles = g.getCyclesRecursive();
    if (cycles.empty())
        cout<<"There are no cycles in the graph"<<endl;
    else
    {
        cout<<"Total number of cycles in the graph: "<<cycles.size()<<endl;
        for (auto &x : cycles)
        {
            for (auto &y : x)
                cout<<y;
            cout<<endl;
        }
    }

    cout<<"Enter the source node"<<endl;
    cin>>src;
    cout<<"Running Dijkstra..."<<endl;
    vector<int> dists = g.graphBFS(src);
    for (int x = 1; x < dists.size(); x++)
    {
        if (dists[x] == INT_MAX)
            cout<<"Distance to node "<<x<<" :IMPOSSIBLE"<<endl;
        else
            cout<<"Distance to node "<<x<<" :"<<dists[x]<<endl;
    }
    cout<<"Running Kruskal Minimum Spanning Tree (MST)..."<<endl;
    g.MST();
    cout<<"Running Dijkstra with MST..."<<endl;
    dists = g.graphBFS(src);
    for (int x = 1; x < dists.size(); x++)
    {
        if (dists[x] == INT_MAX)
            cout<<"Distance to node "<<x<<" :IMPOSSIBLE"<<endl;
        else
            cout<<"Distance to node "<<x<<" :"<<dists[x]<<endl;
    }
    return 0;
}
