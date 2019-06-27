#include <bits/stdc++.h>

using namespace std;


typedef vector<vector<int>> Adj;
typedef long long lli;
typedef priority_queue<pair<lli, int>,vector<pair<lli,int>>,greater<pair<lli,int>>> minHeap;

const lli INFINITY_ = numeric_limits<lli>::max() / 4;

class AStar {
    
    int n_;
    Adj adj_;
    Adj cost_;
    vector<lli> distance_;
    vector<bool> visited_;
	vector<int> workset_;
	vector<lli> potentials_;
    vector<pair<lli,lli>> coords_;

public:
    AStar(int n, Adj adj, Adj cost, vector<pair<lli,lli>> coords)
        : n_(n), adj_(adj), cost_(cost), distance_(vector<lli>(n_, INFINITY_)), visited_(n), coords_(coords), potentials_(vector<lli>(n_, -1))
    {workset_.reserve(n);}

    
    void clear() {
        for (auto &v: workset_) {
            distance_[v] = INFINITY_;
            visited_[v] = false;
			potentials_[v] = -1;
        }
		workset_.clear();
    }

    
    void visit(minHeap& minHeap_forward, int v, lli dist, lli potential_value) {
        
		if (distance_[v] > dist) {
			distance_[v] = dist;
			minHeap_forward.push({distance_[v] + potential_value, v});
			workset_.push_back(v);
		}
    }
	
	lli Potential(int u, int v) {
		if(potentials_[u] == -1) {
			pair<lli, lli> p_u = coords_[u];
			pair<lli, lli> p_v = coords_[v];
			potentials_[u] = sqrt((p_u.first - p_v.first)*(p_u.first - p_v.first) + (p_u.second - p_v.second)*(p_u.second - p_v.second));
		}
		return potentials_[u];
	}
	
	void Process(minHeap& minHeap_forward, int u, int t, vector<vector<int>> &adj, const vector<vector<int>> &cost) {
		for (int i = 0; i < adj[u].size(); ++i) {
			int v = adj[u][i];
			if (visited_[v] != true) {
				int weight = cost[u][i];
				visit(minHeap_forward, v, distance_[u] + weight, Potential(v, t));
			}
		}	
	}


    lli query(int source, int destination) {
        clear();
        minHeap minHeap_forward;
        visit(minHeap_forward, source, 0, Potential(source, destination));
		while (!minHeap_forward.empty()) {
			int v = minHeap_forward.top().second;
            minHeap_forward.pop();
			if (v == destination) {
				if(distance_[destination] != INFINITY_)
					return distance_[destination];
				else
					return -1;
			}
			if (visited_[v] != true) {
				Process(minHeap_forward, v, destination, adj_, cost_);
				visited_[v] = true;
			}	
		}
        return -1;
    }
};

int main() {
    int n, m;
    scanf("%d%d", &n, &m);
    vector<pair<lli,lli>> coords(n);
    for (int i=0;i<n;++i){
        int a, b;
        scanf("%d%d", &a, &b);
        coords[i] = make_pair(a,b);
    }
    Adj adj(n);
    Adj cost(n);
    for (int i=0; i<m; ++i) {
        int u, v, c;
        scanf("%d%d%d", &u, &v, &c);
        adj[u-1].push_back(v-1);
        cost[u-1].push_back(c);
    }

    AStar astar(n, adj, cost, coords);

    int query;
    scanf("%d", &query);
    for (int i=0; i<query; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n", astar.query(u-1, v-1));
    }
}