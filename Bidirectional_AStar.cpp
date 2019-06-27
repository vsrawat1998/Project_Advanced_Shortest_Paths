#include <bits/stdc++.h>
#define fastio() ios_base::sync_with_stdio(false);cin.tie(NULL);cout.tie(NULL)

using namespace std;

typedef pair<long long, long long> Point;
typedef long long lli;
typedef pair<long long, int> Edge;
typedef vector<vector<vector<Edge>>> Adj_List;
typedef vector<vector<lli>> Distance;
typedef priority_queue<Edge, vector<Edge>, greater<Edge>> minHeap;

lli INFI = std::numeric_limits<lli>::max() / 4;

class AStar {
    Adj_List adj;
    Distance distance_;
    vector<Point> coords;
    vector<bool> visited_forward, visited_reverse;
    vector<int> workset;
    minHeap minheap_forward, minheap_reverse;
    int destination_{0}, source_{0};

    void clear() ;
    lli compute_path() ;
    lli euclidian(int, int) ;
    inline lli potential(int) ;
    inline void process(int, minHeap&, int) ;
    inline void relax(minHeap&, lli, int, int, int) ;

public:
    AStar(int) ;
    lli bidirectional_Astar(int s, int t) ;
    inline void add_edge(int u, int v, int cost) ;
    inline void add_coords(int x, int y) ;
};

AStar::AStar(int n) :
    adj(Adj_List(2, vector<vector<Edge>>(n, vector<Edge>()))),
    distance_(2, vector<lli>(n, INFI)),
    visited_forward(n, false),
    visited_reverse(n, false)
    {};

lli AStar::euclidian(int u, int v)  {
    return sqrt((coords[u].first - coords[v].first) * (coords[u].first - coords[v].first)
            +(coords[u].second - coords[v].second) * (coords[u].second - coords[v].second));
}

lli AStar::potential(int u)  {
    return (euclidian(u, destination_) - euclidian(source_, u)) / 2;
}

void AStar::add_coords(int x, int y)  {
    coords.push_back(Point(x, y));
}

void AStar::clear()  {
    for(int u = -1; !minheap_forward.empty(); minheap_forward.pop() ) {
        u = minheap_forward.top().second;
        distance_[1][u] = distance_[0][u] = INFI;
        visited_forward[u] = visited_reverse[u] = 0;
    }

    for(int u = -1; !minheap_reverse.empty(); minheap_reverse.pop() ) {
        u = minheap_reverse.top().second;
        distance_[1][u] = distance_[0][u] = INFI;
        visited_forward[u] = visited_reverse[u] = 0;
    }

    for(auto u: workset) {
        distance_[1][u] = distance_[0][u] = INFI;
        visited_forward[u] = visited_reverse[u] = 0;
    }

    workset.clear();
}

void AStar::add_edge(int u, int v, int cost)  {
    adj[0][u].push_back(Edge(cost, v));
    adj[1][v].push_back(Edge(cost, u));
}

void AStar::relax(minHeap& minheap_forward, lli weight, int v, int u, int side)  {
    lli pot = -potential(u) + potential(v);
    if(side)
        pot*= -1;

    if(distance_[side][v] > distance_[side][u] + weight + pot) {
        distance_[side][v] = distance_[side][u] + weight + pot;
        minheap_forward.push(Edge(distance_[side][v], v));
    }
}

void AStar::process(int u, minHeap& minheap_forward, int side)  {
    minheap_forward.pop();
    for(auto &edge: adj[side][u])
        relax(minheap_forward, edge.first, edge.second, u, side);
}

lli AStar::compute_path()  {
    lli result = INFI;
    for(auto u: workset)
        if(distance_[0][u] + distance_[1][u] < result)
        result = distance_[0][u] + distance_[1][u];
    if(result == INFI)
        return -1;

    return result + potential(source_) -  potential(destination_);
}

lli AStar::bidirectional_Astar(int s, int t)  {
    clear();
    source_ = s, destination_ = t;

    minheap_forward.push(Edge(0, s));
    minheap_reverse.push(Edge(0, t));
    distance_[0][s] = distance_[1][t] = 0;

    do {
        int u = minheap_forward.top().second;
        process(u, minheap_forward, 0);
        visited_forward[u] = true;
        if(visited_reverse[u])
            break;
        workset.push_back(u);

        u = minheap_reverse.top().second;
        process(u, minheap_reverse, 1);
        visited_reverse[u] = true;
        if(visited_forward[u])
            break;
        workset.push_back(u);

    } while(!minheap_forward.empty() && !minheap_reverse.empty());

  return compute_path();
}

int  main() {
	fastio();
    int  n, m, u, v, q;
    cin >> n >> m;
    AStar astar(n);

    for (int  i = 0; i < n; i++) {
        int  x, y;
        cin >> x >> y;
        astar.add_coords(x, y);
    }

    for (int  i = 0; i < m; i++) {
        int  x, y, cost;
        cin >> x >> y >> cost;
        astar.add_edge(x-1, y-1, cost);
    }

    cin >> q;
    while(q--) {
    cin >> u >> v;
    cout << astar.bidirectional_Astar(u-1, v-1) << "\n";
    }
}