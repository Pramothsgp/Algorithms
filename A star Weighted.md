### **A\* Algorithm for Weighted Graph (Adjacency Matrix) in Java & C++**  
Since the input is given as an **adjacency matrix with weights**, we will modify the A\* algorithm to handle this format.

---

## **Java Implementation**
```java
import java.util.*;

class Node implements Comparable<Node> {
    int vertex, g, h, f;
    Node parent;

    public Node(int vertex, Node parent, int g, int h) {
        this.vertex = vertex;
        this.parent = parent;
        this.g = g;
        this.h = h;
        this.f = g + h;
    }

    public int compareTo(Node other) {
        return Integer.compare(this.f, other.f);
    }
}

public class AStarGraph {
    public static void astar(int[][] graph, int start, int end) {
        int n = graph.length;
        PriorityQueue<Node> openList = new PriorityQueue<>();
        Map<Integer, Integer> gCosts = new HashMap<>();
        
        Node startNode = new Node(start, null, 0, heuristic(start, end));
        openList.add(startNode);
        gCosts.put(start, 0);

        while (!openList.isEmpty()) {
            Node current = openList.poll();
            if (current.vertex == end) {
                printPath(current);
                return;
            }

            for (int neighbor = 0; neighbor < n; neighbor++) {
                if (graph[current.vertex][neighbor] > 0) { // Edge exists
                    int newG = current.g + graph[current.vertex][neighbor];

                    if (!gCosts.containsKey(neighbor) || newG < gCosts.get(neighbor)) {
                        gCosts.put(neighbor, newG);
                        Node neighborNode = new Node(neighbor, current, newG, heuristic(neighbor, end));
                        openList.add(neighborNode);
                    }
                }
            }
        }
        System.out.println("No path found.");
    }

    private static int heuristic(int a, int b) {
        return 0; // Since we only have a graph, no coordinates (Dijkstra-like behavior)
    }

    private static void printPath(Node node) {
        List<Integer> path = new ArrayList<>();
        while (node != null) {
            path.add(node.vertex);
            node = node.parent;
        }
        Collections.reverse(path);
        for (int p : path) {
            System.out.print(p + " ");
        }
        System.out.println();
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
        astar(graph, start, end);
    }
}
```

### **Explanation**
- The **graph is given as an adjacency matrix**, where `graph[i][j]` represents the weight between nodes `i` and `j`.
- The **A\*** heuristic is **set to zero** since this behaves more like **Dijkstra’s algorithm**.
- The algorithm uses a **priority queue (min-heap)** to always expand the node with the lowest **`f = g + h`** value.
- **Path is reconstructed by backtracking through parent nodes**.

---

## **C++ Implementation**
```cpp
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>

using namespace std;

struct Node {
    int vertex, g, h, f;
    Node* parent;

    Node(int v, Node* p, int g_val, int h_val) {
        vertex = v;
        parent = p;
        g = g_val;
        h = h_val;
        f = g + h;
    }
};

struct Compare {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;
    }
};

void astar(vector<vector<int>>& graph, int start, int end) {
    int n = graph.size();
    priority_queue<Node*, vector<Node*>, Compare> openList;
    unordered_map<int, int> gCosts;

    Node* startNode = new Node(start, nullptr, 0, 0);
    openList.push(startNode);
    gCosts[start] = 0;

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        if (current->vertex == end) {
            vector<int> path;
            while (current) {
                path.push_back(current->vertex);
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            for (int p : path) {
                cout << p << " ";
            }
            cout << endl;
            return;
        }

        for (int neighbor = 0; neighbor < n; neighbor++) {
            if (graph[current->vertex][neighbor] > 0) { // Edge exists
                int newG = current->g + graph[current->vertex][neighbor];

                if (gCosts.find(neighbor) == gCosts.end() || newG < gCosts[neighbor]) {
                    gCosts[neighbor] = newG;
                    Node* neighborNode = new Node(neighbor, current, newG, 0);
                    openList.push(neighborNode);
                }
            }
        }
    }
    cout << "No path found." << endl;
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
    astar(graph, start, end);
    return 0;
}
```

---

### **Example Input**
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

### **Output**
```
0 2 4
```

### **Path Explanation**
The **shortest path from node `0` to `4`** considering edge weights is:
- Start at **0**
- Move to **2** (cost `3`)
- Move to **4** (cost `2`)
- **Total Cost = 3 + 2 = 5** (Optimal)

---

### **Why Does This Work Like Dijkstra?**
- Since we don't have a **coordinate-based heuristic**, `h(n) = 0`, meaning **A\* effectively behaves like Dijkstra's Algorithm**.
- A\* still guarantees the shortest path since it **only expands nodes with the lowest known cost**.