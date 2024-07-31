#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

class SupplyChainNetwork {
public:
    SupplyChainNetwork(int num_nodes);
    void addEdge(int start, int end, int cost);
    vector<int> dijkstra(int start);
    void setDemandForecast(const vector<int>& demand_forecast);
    int calculateCost(const vector<int>& distances);
    pair<vector<int>, int> optimizeNetwork(int start);

private:
    int num_nodes;
    vector<vector<pair<int, int>>> adj_list; // adjacency list for the graph
    vector<int> demand_forecast;
};

SupplyChainNetwork::SupplyChainNetwork(int num_nodes) : num_nodes(num_nodes) {
    adj_list.resize(num_nodes);
}

void SupplyChainNetwork::addEdge(int start, int end, int cost) {
    adj_list[start].emplace_back(end, cost);
    adj_list[end].emplace_back(start, cost); // Assuming undirected graph
}

vector<int> SupplyChainNetwork::dijkstra(int start) {
    vector<int> distances(num_nodes, INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.emplace(0, start);
    distances[start] = 0;

    while (!pq.empty()) {
        int current_dist = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (current_dist > distances[u]) continue;

        for (const auto& neighbor : adj_list[u]) {
            int v = neighbor.first;
            int cost = neighbor.second;
            int new_dist = current_dist + cost;

            if (new_dist < distances[v]) {
                distances[v] = new_dist;
                pq.emplace(new_dist, v);
            }
        }
    }
    return distances;
}

void SupplyChainNetwork::setDemandForecast(const vector<int>& forecast) {
    demand_forecast = forecast;
}

int SupplyChainNetwork::calculateCost(const vector<int>& distances) {
    int total_cost = 0;
    for (size_t i = 0; i < distances.size(); ++i) {
        total_cost += distances[i] * demand_forecast[i];
    }
    return total_cost;
}

pair<vector<int>, int> SupplyChainNetwork::optimizeNetwork(int start) {
    vector<int> distances = dijkstra(start);
    int cost = calculateCost(distances);
    return {distances, cost};
}

int main() {
    int num_nodes = 5;
    SupplyChainNetwork network(num_nodes);

    // Add edges with (start, end, cost)
    network.addEdge(0, 1, 10);
    network.addEdge(1, 2, 20);
    network.addEdge(2, 3, 30);
    network.addEdge(3, 4, 40);
    network.addEdge(0, 4, 100);

    // Set demand forecast
    vector<int> demand_forecast = {5, 10, 15, 20, 25};
    network.setDemandForecast(demand_forecast);

    // Optimize network starting from node 0
    pair<vector<int>, int> result = network.optimizeNetwork(0);
    vector<int> distances = result.first;
    int cost = result.second;

    cout << "Shortest distances from node 0:" << endl;
    for (int d : distances) {
        cout << d << " ";
    }
    cout << endl;

    cout << "Total cost: " << cost << endl;

    return 0;
}
