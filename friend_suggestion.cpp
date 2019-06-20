#include <bits/stdc++.h>

using namespace std;

// External vector of size 2 - for forward and backward search.
// Internal 2-dimensional vector is vector of adjacency lists for each node.
typedef vector<vector<vector<int>>> Adj;

// Distances can grow out of int type
typedef long long lli;

typedef pair<lli,int> PII;

// Vector of two priority queues - for forward and backward searches.
// Each priority queue stores the closest unprocessed node in its head.
typedef vector<priority_queue<pair<lli, int>,vector<pair<lli,int>>,greater<pair<lli,int>>>> minHeap;

const lli INFI = numeric_limits<lli>::max() / 4;

class Bidijkstra {
    // Number of nodes
    int n_;
    // Graph adj_[0] and cost_[0] correspond to the initial graph,
    // adj_[1] and cost_[1] correspond to the reversed graph.
    // Graphs are stored as vectors of adjacency lists corresponding
    // to nodes.
    // Adjacency list itself is stored in adj_, and the corresponding
    // edge costs are stored in cost_.
    Adj adj_;
    Adj cost_;
    // distance_[0] stores distances for the forward search,
    // and distance_[1] stores distances for the backward search.
    vector<vector<lli>> distance_;
    // visit_front stores the vertices that are discovered in the graph
    vector<bool> visit_front;
    // visit_back stores the vertices that are discovered in the reversed graph
    vector<bool> visit_back;

public:
    Bidijkstra(int n, Adj adj, Adj cost)
        : n_(n), adj_(adj), cost_(cost), distance_(2, vector<lli>(n, INFI)),visit_front(n,false),visit_back(n,false)
    { 
    }
    // Returns the distance from s to t in the graph.
    lli query(int s, int t) {
        
        // q stores the edge costs of graph in index 0
        // and reversed graph in index 1.
        minHeap q(2);

        // proc stores the vertices discovered in forward search
        vector<int> proc;
        // proc_rev stores the vertices discovered in reverse search
        vector<int> proc_rev;

        distance_[0][s] = 0;
        distance_[1][t] = 0;

        // Defining the distance between the source and the required vertex as infinity
        // in case there are no paths from s to t
        lli shortestDistance = INFI;
        q[0].push({distance_[0][s],s});
        q[1].push({distance_[1][t],t});
        
        int current_node;
        PII pii;
        // In both directions, Djikstra runs in the same manner, i.e.,
        // select the edge with smallest edge cost and add it to the distances.
        
        // For every node discovered in both directions, we have to sum the 
        // distances from s to this node and t to this node.
        // Only summing up the distances for the middle node will not be sufficient.
        // For example,
        // 1 connected to 2 with cost 3.
        // 2 connected to 3 with cost 4.
        // 1 connected to 3 with cost 6.
        // Bidirection Dijkstra will choose the same way as the normal Dijsktra algorithm
        // therefore 1-->2 and 2-->3 will result in a path with cost 7 but the shortest
        // path is 1-->3 with cost 6
        while(!q[0].empty() && !q[1].empty()){

            // Djikstra in forward direction

            pii = q[0].top();
            q[0].pop();
            current_node = pii.second;
            proc.push_back(current_node);
            for(int i = 0;i<adj_[0][current_node].size();i++){
                if(distance_[0][adj_[0][current_node][i]] > distance_[0][current_node] + (lli)cost_[0][current_node][i]){
                    distance_[0][adj_[0][current_node][i]] = distance_[0][current_node] + (lli)cost_[0][current_node][i];
                    q[0].push({distance_[0][adj_[0][current_node][i]],adj_[0][current_node][i]});
                    proc.push_back(adj_[0][current_node][i]);
                }
            }
            
            visit_front[current_node] = true;

            // Summing the distances from s to current node and t to current node.
            if(shortestDistance > distance_[0][current_node] + distance_[1][current_node])
                shortestDistance = distance_[0][current_node] + distance_[1][current_node];

            // If the current node is already a part of the reverse search
            // then we are already done with the path.
            if(visit_back[current_node])
                break;
            

            // Djikstra in reverse direction
            pii = q[1].top();
            q[1].pop();
            current_node = pii.second;
            
            proc_rev.push_back(current_node);
            for(int i = 0;i<adj_[1][current_node].size();i++){
                if(distance_[1][adj_[1][current_node][i]] > distance_[1][current_node] + (lli)cost_[1][current_node][i]){
                    distance_[1][adj_[1][current_node][i]] = distance_[1][current_node] + (lli)cost_[1][current_node][i];
                    q[1].push({distance_[1][adj_[1][current_node][i]],adj_[1][current_node][i]});
                    proc_rev.push_back(adj_[1][current_node][i]);
                }
            }
            
            // Summing the distances from s to current node and t to current node.
            if(shortestDistance > distance_[0][current_node] + distance_[1][current_node])
                shortestDistance = distance_[0][current_node] + distance_[1][current_node];
            visit_back[current_node] = true;

            // If the current node is already a part of the forward search
            // then we are already done with the path.
            if(visit_front[current_node])
                break;

        }
        // The values which were changed in the course of the algorithm are reverted to 
        // initial values of distance_, visit_front and visit_back.
        // To get them ready for the next iteration
        for(auto it = proc.begin();it!=proc.end();it++){
            distance_[0][*it] = INFI;
            visit_front[*it] = false;
        }
        for(auto it = proc_rev.begin();it!=proc_rev.end();it++){
            distance_[1][*it] = INFI;
            visit_back[*it] = false;
        }

        // If the shortest distance remains infinity, then we can return -1, i.e., t cannot be reached from s.
        if(shortestDistance == INFI)
            return -1;
        
        return shortestDistance;
    }
};

int main() {
    int n, m;
    scanf("%d%d", &n, &m);
    Adj adj(2, vector<vector<int>>(n));
    Adj cost(2, vector<vector<int>>(n));
    for (int i=0; i<m; ++i) {
        int u, v, c;
        scanf("%d%d%d", &u, &v, &c);
        adj[0][u-1].push_back(v-1);
        cost[0][u-1].push_back(c);
        adj[1][v-1].push_back(u-1);
        cost[1][v-1].push_back(c);
    }

    Bidijkstra bidij(n, adj, cost);

    int t;
    scanf("%d", &t);
    for (int i=0; i<t; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n", bidij.query(u-1, v-1));
    }
}
