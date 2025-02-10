

### **Java Implementation**
```java
import java.util.*;

class Node implements Comparable<Node> {
    int x, y, g, h, f;
    Node parent;

    public Node(int x, int y, Node parent) {
        this.x = x;
        this.y = y;
        this.parent = parent;
    }

    public int compareTo(Node other) {
        return Integer.compare(this.f, other.f);
    }
}

public class AStar {
    static int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    public static List<int[]> astar(int[][] grid, int[] start, int[] end) {
        PriorityQueue<Node> openList = new PriorityQueue<>();
        Set<String> closedSet = new HashSet<>();

        Node startNode = new Node(start[0], start[1], null);
        startNode.g = 0;
        startNode.h = heuristic(start, end);
        startNode.f = startNode.g + startNode.h;
        openList.add(startNode);

        while (!openList.isEmpty()) {
            Node current = openList.poll();
            String key = current.x + "," + current.y;
            closedSet.add(key);

            if (current.x == end[0] && current.y == end[1]) {
                return reconstructPath(current);
            }

            for (int[] dir : directions) {
                int nx = current.x + dir[0], ny = current.y + dir[1];
                if (nx < 0 || ny < 0 || nx >= grid.length || ny >= grid[0].length || grid[nx][ny] == 1) continue;

                String neighborKey = nx + "," + ny;
                if (closedSet.contains(neighborKey)) continue;

                Node neighbor = new Node(nx, ny, current);
                neighbor.g = current.g + 1;
                neighbor.h = heuristic(new int[]{nx, ny}, end);
                neighbor.f = neighbor.g + neighbor.h;

                openList.add(neighbor);
            }
        }
        return new ArrayList<>(); // No path found
    }

    private static int heuristic(int[] a, int[] b) {
        return Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]); // Manhattan distance
    }

    private static List<int[]> reconstructPath(Node node) {
        List<int[]> path = new ArrayList<>();
        while (node != null) {
            path.add(new int[]{node.x, node.y});
            node = node.parent;
        }
        Collections.reverse(path);
        return path;
    }

    public static void main(String[] args) {
        int[][] grid = {
            {0, 0, 0, 0, 1},
            {1, 1, 0, 1, 0},
            {0, 0, 0, 0, 0},
            {0, 1, 1, 1, 0},
            {0, 0, 0, 0, 0}
        };
        int[] start = {0, 0};
        int[] end = {4, 4};

        List<int[]> path = astar(grid, start, end);
        for (int[] p : path) {
            System.out.println(Arrays.toString(p));
        }
    }
}
```

---

### **C++ Implementation**
```cpp
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <cmath>
using namespace std;

struct Node {
    int x, y, g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), parent(parent), g(0), h(0), f(0) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

int heuristic(pair<int, int> a, pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

vector<pair<int, int>> astar(vector<vector<int>>& grid, pair<int, int> start, pair<int, int> end) {
    priority_queue<Node, vector<Node>, greater<Node>> openList;
    set<pair<int, int>> closedSet;
    vector<vector<int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    Node* startNode = new Node(start.first, start.second);
    startNode->g = 0;
    startNode->h = heuristic(start, end);
    startNode->f = startNode->g + startNode->h;
    openList.push(*startNode);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();
        closedSet.insert({current.x, current.y});

        if (current.x == end.first && current.y == end.second) {
            vector<pair<int, int>> path;
            Node* temp = &current;
            while (temp) {
                path.push_back({temp->x, temp->y});
                temp = temp->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto& dir : directions) {
            int nx = current.x + dir[0], ny = current.y + dir[1];

            if (nx < 0 || ny < 0 || nx >= grid.size() || ny >= grid[0].size() || grid[nx][ny] == 1) continue;
            if (closedSet.find({nx, ny}) != closedSet.end()) continue;

            Node* neighbor = new Node(nx, ny, new Node(current.x, current.y, current.parent));
            neighbor->g = current.g + 1;
            neighbor->h = heuristic({nx, ny}, end);
            neighbor->f = neighbor->g + neighbor->h;

            openList.push(*neighbor);
        }
    }
    return {}; // No path found
}

int main() {
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 1},
        {1, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };
    pair<int, int> start = {0, 0};
    pair<int, int> end = {4, 4};

    vector<pair<int, int>> path = astar(grid, start, end);
    for (auto& p : path) {
        cout << "(" << p.first << ", " << p.second << ") ";
    }
    cout << endl;
    return 0;
}
```

---

### **A\* Algorithm Explanation (Java & C++)**  
A\* (A-star) is an efficient pathfinding algorithm used in AI, robotics, and game development. It finds the shortest path from a start point to an end point in a grid while considering obstacles. It combines two cost functions:  

1. **G(n):** The actual cost from the start node to the current node.  
2. **H(n):** The estimated heuristic cost from the current node to the goal (often Manhattan Distance).  
3. **F(n) = G(n) + H(n):** The total estimated cost.  

---

## **Java Implementation Breakdown**  
### **1. Node Class**
This class represents each position in the grid.
```java
class Node implements Comparable<Node> {
    int x, y, g, h, f;
    Node parent;

    public Node(int x, int y, Node parent) {
        this.x = x;
        this.y = y;
        this.parent = parent;
    }

    public int compareTo(Node other) {
        return Integer.compare(this.f, other.f);
    }
}
```
- `x, y`: Represents the coordinates in the grid.  
- `g`: Cost from start to the current node.  
- `h`: Heuristic estimate to the goal.  
- `f`: Total cost (g + h).  
- `parent`: Keeps track of the previous node (for reconstructing the path).  
- Implements `Comparable<Node>` so we can use it in a **priority queue** based on `f`.  

---

### **2. Heuristic Function**
The heuristic function **estimates** the distance to the goal. The Manhattan Distance is used:
```java
private static int heuristic(int[] a, int[] b) {
    return Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);
}
```
This works well for a grid where movement is only allowed in four directions.

---

### **3. A\* Algorithm Execution**
```java
public static List<int[]> astar(int[][] grid, int[] start, int[] end) {
```
- `grid`: The input matrix where `0` represents walkable areas and `1` represents obstacles.  
- `start`: The starting position `[x, y]`.  
- `end`: The goal position `[x, y]`.  

#### **Priority Queue (Open List)**
```java
PriorityQueue<Node> openList = new PriorityQueue<>();
Set<String> closedSet = new HashSet<>();
```
- `openList`: A priority queue to always process the node with the lowest `f` first.  
- `closedSet`: Stores visited nodes.  

#### **Processing Nodes**
```java
Node startNode = new Node(start[0], start[1], null);
startNode.g = 0;
startNode.h = heuristic(start, end);
startNode.f = startNode.g + startNode.h;
openList.add(startNode);
```
- Create the **starting node** with `g = 0` and calculate `h` using the heuristic.  
- Add it to `openList`.  

#### **Exploring Neighbors**
```java
while (!openList.isEmpty()) {
    Node current = openList.poll();
    closedSet.add(current.x + "," + current.y);
```
- Pop the **node with the lowest f** from `openList`.  
- If it's the goal, reconstruct the path.  
- Otherwise, explore its **4 neighboring nodes (left, right, up, down)**.

```java
for (int[] dir : directions) {
    int nx = current.x + dir[0], ny = current.y + dir[1];

    if (nx < 0 || ny < 0 || nx >= grid.length || ny >= grid[0].length || grid[nx][ny] == 1) continue;
```
- Skip out-of-bounds and obstacles.  

```java
Node neighbor = new Node(nx, ny, current);
neighbor.g = current.g + 1;
neighbor.h = heuristic(new int[]{nx, ny}, end);
neighbor.f = neighbor.g + neighbor.h;

openList.add(neighbor);
```
- Calculate `g, h, f` and push the neighbor to the priority queue.

#### **Path Reconstruction**
When we reach the goal, we reconstruct the path:
```java
private static List<int[]> reconstructPath(Node node) {
    List<int[]> path = new ArrayList<>();
    while (node != null) {
        path.add(new int[]{node.x, node.y});
        node = node.parent;
    }
    Collections.reverse(path);
    return path;
}
```
- Start from the goal node and backtrack using `parent` to reconstruct the full path.

---

## **C++ Implementation Breakdown**
The logic is the same as Java, but written in **C++ style**.

### **1. Node Struct**
```cpp
struct Node {
    int x, y, g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), parent(parent), g(0), h(0), f(0) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};
```
- Defines a **struct** for nodes.
- Uses `operator>` to prioritize lower `f` values in **priority queue**.

---

### **2. Heuristic Function**
```cpp
int heuristic(pair<int, int> a, pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}
```
- Same **Manhattan Distance** heuristic as in Java.

---

### **3. A\* Algorithm Execution**
```cpp
priority_queue<Node, vector<Node>, greater<Node>> openList;
set<pair<int, int>> closedSet;
```
- `priority_queue`: Min-heap for selecting nodes with the **lowest f**.
- `set`: Keeps track of visited nodes.

#### **Processing the Start Node**
```cpp
Node* startNode = new Node(start.first, start.second);
startNode->g = 0;
startNode->h = heuristic(start, end);
startNode->f = startNode->g + startNode->h;
openList.push(*startNode);
```
- Initializes the start node and adds it to the open list.

#### **Exploring Neighbors**
```cpp
for (auto& dir : directions) {
    int nx = current.x + dir[0], ny = current.y + dir[1];

    if (nx < 0 || ny < 0 || nx >= grid.size() || ny >= grid[0].size() || grid[nx][ny] == 1) continue;
```
- Ensures movement stays within the **grid boundaries** and avoids obstacles.

```cpp
Node* neighbor = new Node(nx, ny, new Node(current.x, current.y, current.parent));
neighbor->g = current.g + 1;
neighbor->h = heuristic({nx, ny}, end);
neighbor->f = neighbor->g + neighbor->h;

openList.push(*neighbor);
```
- Computes `g, h, f` and pushes the neighbor to the queue.

---

### **4. Path Reconstruction**
```cpp
vector<pair<int, int>> path;
Node* temp = &current;
while (temp) {
    path.push_back({temp->x, temp->y});
    temp = temp->parent;
}
reverse(path.begin(), path.end());
```
- Constructs the shortest path by **backtracking from the goal node**.

---

## **Summary of Java vs C++**
| Feature         | Java                         | C++                          |
|---------------|----------------------------|-----------------------------|
| **Data Structures** | `PriorityQueue<Node>` | `priority_queue<Node>` |
| **Set for Closed List** | `HashSet<String>` | `set<pair<int, int>>` |
| **Node Representation** | `class Node` | `struct Node` |
| **Heuristic** | `private static int heuristic()` | `int heuristic()` |
| **Memory Management** | Garbage Collection | `new` and `delete` (manual) |

Both implementations work efficiently with **O(N log N)** time complexity due to the **priority queue**.

Would you like any modifications or further clarifications? 🚀