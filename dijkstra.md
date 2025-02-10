Yes! **Dijkstra’s Algorithm** is perfectly suited for finding the **shortest path in a weighted graph**. While A\* uses a heuristic to guide the search, **Dijkstra’s Algorithm guarantees the shortest path by always expanding the lowest-cost node first**.  

---

## **Dijkstra’s Algorithm for Finding the Path (Java & C++)**
Here, we’ll modify Dijkstra’s Algorithm to **not only find the shortest distance but also reconstruct the path**.

---

## **Java Implementation**
```java
import java.util.*;

class DijkstraPath {
    static class Node implements Comparable<Node> {
        int vertex, cost;
        
        public Node(int vertex, int cost) {
            this.vertex = vertex;
            this.cost = cost;
        }
        
        public int compareTo(Node other) {
            return Integer.compare(this.cost, other.cost);
        }
    }

    public static void dijkstra(int[][] graph, int start, int end) {
        int n = graph.length;
        int[] dist = new int[n];  
        int[] parent = new int[n];  
        Arrays.fill(dist, Integer.MAX_VALUE);
        Arrays.fill(parent, -1);
        
        PriorityQueue<Node> pq = new PriorityQueue<>();
        dist[start] = 0;
        pq.add(new Node(start, 0));

        while (!pq.isEmpty()) {
            Node current = pq.poll();
            int u = current.vertex;

            if (u == end) break;  // Stop early if we reach the destination

            for (int v = 0; v < n; v++) {
                if (graph[u][v] > 0) {  // Edge exists
                    int newDist = dist[u] + graph[u][v];
                    if (newDist < dist[v]) {
                        dist[v] = newDist;
                        parent[v] = u;
                        pq.add(new Node(v, newDist));
                    }
                }
            }
        }

        printPath(parent, start, end, dist[end]);
    }

    private static void printPath(int[] parent, int start, int end, int cost) {
        if (parent[end] == -1) {
            System.out.println("No path found.");
            return;
        }

        List<Integer> path = new ArrayList<>();
        for (int at = end; at != -1; at = parent[at]) {
            path.add(at);
        }
        Collections.reverse(path);

        System.out.println("Shortest Path: " + path);
        System.out.println("Total Cost: " + cost);
    }

    public static void main(String[] args) {
        int[][] graph = {
            {0, 10, 3, 0, 0},
            {10, 0, 1, 2, 0},
            {3, 1, 0, 8, 2},
            {0, 2, 8, 0, 4},
            {0, 0, 2, 4, 0}
        };

        int start = 0, end = 4;
        dijkstra(graph, start, end);
    }
}
```

---

## **C++ Implementation**
```cpp
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>

using namespace std;

struct Node {
    int vertex, cost;
    Node(int v, int c) : vertex(v), cost(c) {}
};

struct Compare {
    bool operator()(Node a, Node b) {
        return a.cost > b.cost;
    }
};

void dijkstra(vector<vector<int>>& graph, int start, int end) {
    int n = graph.size();
    vector<int> dist(n, INT_MAX);
    vector<int> parent(n, -1);
    priority_queue<Node, vector<Node>, Compare> pq;

    dist[start] = 0;
    pq.push(Node(start, 0));

    while (!pq.empty()) {
        int u = pq.top().vertex;
        pq.pop();

        if (u == end) break;

        for (int v = 0; v < n; v++) {
            if (graph[u][v] > 0) {
                int newDist = dist[u] + graph[u][v];
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    parent[v] = u;
                    pq.push(Node(v, newDist));
                }
            }
        }
    }

    if (parent[end] == -1) {
        cout << "No path found." << endl;
        return;
    }

    vector<int> path;
    for (int at = end; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    cout << "Shortest Path: ";
    for (int node : path) {
        cout << node << " ";
    }
    cout << "\nTotal Cost: " << dist[end] << endl;
}

int main() {
    vector<vector<int>> graph = {
        {0, 10, 3, 0, 0},
        {10, 0, 1, 2, 0},
        {3, 1, 0, 8, 2},
        {0, 2, 8, 0, 4},
        {0, 0, 2, 4, 0}
    };

    int start = 0, end = 4;
    dijkstra(graph, start, end);
    return 0;
}
```

---

## **Example Input**
```plaintext
Graph (Adjacency Matrix):
  0   10   3    0    0
 10    0   1    2    0
  3    1   0    8    2
  0    2   8    0    4
  0    0   2    4    0

Start Node: 0
End Node: 4
```

## **Output**
```
Shortest Path: [0, 2, 4]
Total Cost: 5
```

---

## **Explanation of the Path**
1. **Start at Node `0`**
   - Possible edges: `(0 → 1, cost 10)`, `(0 → 2, cost 3)`, `(0 → 3, cost ∞)`, `(0 → 4, cost ∞)`
   - Picks **Node `2` (cost 3)** since it’s the cheapest.
  
2. **Move to Node `2`**
   - Possible edges: `(2 → 1, cost 1)`, `(2 → 3, cost 8)`, `(2 → 4, cost 2)`
   - Picks **Node `4` (cost 3 + 2 = 5)** since it’s the cheapest.

3. **Reached Node `4`**  
   - **Total cost = 5**
   - **Path: `[0 → 2 → 4]`**

---

### **Why Use Dijkstra Instead of A\*?**
| Feature       | Dijkstra                     | A\* Algorithm  |
|--------------|----------------------------|---------------|
| Uses Heuristic | ❌ No                     | ✅ Yes        |
| Guarantees Shortest Path | ✅ Yes | ✅ Yes (if heuristic is admissible) |
| Best for Grid-based Search | ❌ No  | ✅ Yes |
| Best for Graph with Weights | ✅ Yes | ❌ No (without good heuristic) |

Since **we only have an adjacency matrix and not spatial coordinates**, **Dijkstra is the better choice** because A\* without a heuristic behaves like Dijkstra anyway.

---

## **Final Thoughts**
- If your input is **just an adjacency matrix with weights**, **Dijkstra is more efficient** than A\*.
- If your graph has **grid-like spatial data**, A\* is better **with a proper heuristic**.
- Both algorithms can **find the path and cost efficiently**.