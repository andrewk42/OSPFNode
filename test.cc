#include <iostream>
#include <map>

using namespace std;

struct Node {
    map<unsigned int, unsigned int> link_costs;
    unsigned int cost, next_hop;
};

unsigned int getMinimumCost(map<unsigned int, Node *> Q) {
    unsigned int min_id = 0, min_cost = 1000;

    for (map<unsigned int, Node *>::iterator it = Q.begin(); it != Q.end(); it++) {
        if (it->second->cost < min_cost) {
            min_cost = it->second->cost;
            min_id = it->first;
        }
    }

    return min_id;
}

int main() {
    cout << "Starting" << endl;

    Node *n = NULL;
    map<unsigned int, Node *> C, Q;

    // Create node 1
    n = new Node();
    n->link_costs.insert(make_pair(2, 1));
    n->link_costs.insert(make_pair(5, 5));
    n->cost = 0;
    Q.insert(make_pair(1, n));

    // Create node 2
    n = new Node();
    n->link_costs.insert(make_pair(1, 1));
    n->link_costs.insert(make_pair(3, 2));
    n->link_costs.insert(make_pair(5, 6));
    n->cost = 0xFFFFFFFF;
    Q.insert(make_pair(2, n));

    // Create node 3
    n = new Node();
    n->link_costs.insert(make_pair(2, 2));
    n->link_costs.insert(make_pair(4, 3));
    n->link_costs.insert(make_pair(5, 7));
    n->cost = 0xFFFFFFFF;
    Q.insert(make_pair(3, n));

    // Create node 4
    n = new Node();
    n->link_costs.insert(make_pair(5, 4));
    n->link_costs.insert(make_pair(3, 3));
    n->cost = 0xFFFFFFFF;
    Q.insert(make_pair(4, n));

    // Create node 5
    n = new Node();
    n->link_costs.insert(make_pair(1, 5));
    n->link_costs.insert(make_pair(2, 6));
    n->link_costs.insert(make_pair(3, 7));
    n->link_costs.insert(make_pair(4, 4));
    n->cost = 0xFFFFFFFF;
    Q.insert(make_pair(5, n));

    /*for (map<unsigned int, Node *>::iterator it = Q.begin(); it != Q.end(); it++) {
        cout << "Data for node " << it->first << ":" << endl;
        for (map<unsigned int, unsigned int>::iterator inner_it = it->second->link_costs.begin(); inner_it != it->second->link_costs.end(); inner_it++) {
            cout << "Has link to node " << inner_it->first << " with weight " << inner_it->second << endl;
        }
    }*/

    /*for (map<unsigned int, unsigned int>::iterator it = cost.begin(); it != cost.end(); it++) {
        cout << "Cost from s to " << it->first << " is " << it->second << endl;
    }*/

    unsigned int next_node, new_cost;
    Node *origin = Q[1];

    while (!Q.empty()) {
        next_node = getMinimumCost(Q);
        n = Q[next_node];
        cout << "Got next_node " << next_node << " with current cost " << n->cost << endl;
        Q.erase(next_node);
        C.insert(make_pair(next_node, n));

        for (map<unsigned int, unsigned int>::iterator it = n->link_costs.begin(); it != n->link_costs.end(); it++) {
            new_cost = n->cost + it->second;
            cout << "For next_node " << next_node << ", got new cost " << new_cost << endl;
            if (Q.count(it->first) > 0 && new_cost < Q[it->first]->cost) {
                Q[it->first]->cost = new_cost;
                cout << "Set node " << it->first << "'s cost to " << Q[it->first]->cost << endl;
                if (origin->link_costs.count(it->first) > 0) {
                    Q[it->first]->next_hop = it->first;
                } else {
                    Q[it->first]->next_hop = n->next_hop;
                }
                cout << "Setting node " << it->first << "'s next_hop to " << Q[it->first]->next_hop << endl;
            }
        }
        cout << "-----------Done iteration-----------" << endl;
    }

    cout << "All done! The shortest path from R1 to..." << endl;
    for (map<unsigned int, Node *>::iterator it = C.begin(); it != C.end(); it++) {
        cout << "R" << it->first << " has cost " << it->second->cost << " and next_hop " << it->second->next_hop << endl;
    }
}
