#include "graph.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <functional>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <utility>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  directed = directionalEdges;
  eCount = 0;
}

// destructor
Graph::~Graph() = default;

// @return total number of vertices
int Graph::verticesSize() const { return vertices.size(); }

// @return total number of edges
int Graph::edgesSize() const { return eCount; }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  // For each Vertex.
  for (auto &v : vertices) {
    // If we find the string "Label".
    if (v.getID() == label) {
      // Return the size of the list of edges.
      return v.getEdges().size();
    }
  }
  return -1;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  // Check if Vertex already exist.
  bool check = contains(label);
  if (check) {
    return false;
  }
  // If it does not exist create a new Vertex and push it onto the Adjacency
  // List.
  Vertex v;
  v.setID(label);
  vertices.push_back(v);
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  for (auto &v : vertices) {
    if (v.getID() == label) {
      return true;
    }
  }
  return false;
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  stringstream temp;
  temp << "";
  for (auto &v : vertices) {
    if (v.getID() == label) {
      // I am sorting the edge here but I realized later I could have put it
      // into a PriQue.
      list<Edge> edge = v.getEdges();
      edge.sort([](const Edge &edge1, const Edge &edge2) {
        if (edge1.weight == edge2.weight) {
          return edge1 < edge2;
        }
        return edge1.weight < edge2.weight;
      });

      for (auto &e : edge) {
        temp << e.getDestination() << "(" << e.getWeight() << ")";
        if (e.getDestination() != edge.back().getDestination()) {
          temp << ",";
        }
      }
    }
  }
  return temp.str();
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  // If A ---> A return false. no loops aloud.
  if (from == to) {
    return false;
  }

  // Check if not in graph then adds.
  add(from);
  add(to);

  bool successful = false;

  // Checks if edge alread exist.
  if (!checkIfEdgeExist(from, to)) {
    for (auto &v : vertices) {
      if (v.getID() == from) {
        Edge e(to, weight);
        v.edges.push_back(e);
        successful = true;
        eCount++;
      } else if (!directed &&
                 v.getID() == to) { // If not directed connect to --> from.
        Edge e1(from, weight);
        v.edges.push_back(e1);
      }
    }
  }
  return successful;
}

bool Graph::disconnect(const string &from, const string &to) {
  bool check = checkIfEdgeExist(from, to);
  if (check) {
    for (auto &v : vertices) {
      if (v.getID() == from) {
        for (auto edge = v.edges.begin(); edge != v.edges.end(); edge++) {
          if (edge->getDestination() == to) {
            v.edges.erase(edge);
            eCount--;
            break;
          }
        }
      }
      if (!directed) {
        if (v.getID() == to) {
          for (auto edge = v.edges.begin(); edge != v.edges.end(); edge++) {
            if (edge->getDestination() == from) {
              v.edges.erase(edge);
              break;
            }
          }
        }
      }
    }
    return true;
  }
  return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  // Checking if its in graph.
  bool check = contains(startLabel);
  if (!check) {
    return;
  }

  // Putting list into map.
  map<string, list<Edge>> adj;
  for (auto &v : vertices) {
    adj[v.getID()] = v.getEdges();
  }

  // Keeps track of previously visited nodes.
  map<string, bool> seen;

  // Stack to traverse the graph.
  stack<string> myStack;

  // Push onto stack and mark as visited.
  myStack.push(startLabel);
  seen[startLabel] = true;
  vector<string> temp;
  visit(startLabel);

  while (!myStack.empty()) {
    string s = myStack.top();
    myStack.pop();
    seen[s] = true;
    for (const auto &edge : adj[s]) {
      string v = edge.getDestination();
      if (!seen[v]) {
        seen[v] = true;
        temp.push_back(v);
        myStack.push(v);
      }
    }
  }
  sort(temp.begin(), temp.end());
  for (const auto &t : temp) {
    visit(t);
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  bool check = contains(startLabel);
  if (!check) {
    return;
  }

  map<string, list<Edge>> adj;
  for (auto &v : vertices) {
    adj[v.getID()] = v.getEdges();
  }

  map<string, bool> seen;
  queue<string> q;
  vector<string> temp;
  seen[startLabel] = true;
  q.push(startLabel);
  visit(startLabel);

  while (!q.empty()) {

    string s = q.front();
    q.pop();

    for (const auto &edge : adj[s]) {
      string v = edge.getDestination();
      if (!seen[v]) {
        temp.push_back(v);
        seen[v] = true;
      }
    }
    sort(temp.begin(), temp.end());
    for (const string &t : temp) {
      q.push(t);
      visit(t);
    }
    temp.clear();
  }
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  // Stores Vertex : Weight
  map<string, int> weights;
  // Stores the shortest path in pairs.
  map<string, string> previous;
  // previously visited Vertices.
  map<string, bool> visited;
  // Converting list to map.
  map<string, list<Edge>> adj;
  for (auto &v : vertices) {
    adj[v.getID()] = v.getEdges();
  }
  // Creating a pair of int, string so that my PriQue will sort by int (weight).
  typedef pair<int, string> iPair; // NOLINT
  priority_queue<iPair, vector<iPair>, greater<iPair>> pq;

  // Creating my starting point and its weight as 0.
  pq.push(make_pair(0, startLabel));

  while (!pq.empty()) {
    string u = pq.top().second;
    pq.pop();

    for (const auto &edge : adj[u]) {
      string v = edge.getDestination();
      int w = edge.getWeight();

      if (weights.find(v) == weights.end()) {
        weights[v] = INT_MAX;
      }

      // If not visited, and if INT_MAX update the weight and its path and push
      // onto the PriQue.
      if (!visited[u] && weights[v] > weights[u] + w) {
        previous[v] = u;
        weights[v] = weights[u] + w;
        pq.push(make_pair(weights[v], v));
      }
    }
    visited[u] = true;
  }
  // Removing the starting point to pass the output for test.
  weights.erase(startLabel);
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    return -1;
  }
  int result = 0;
  typedef pair<int, string> iPair; // NOLINT
  priority_queue<iPair, vector<iPair>, greater<iPair>> pq;
  map<string, string> path;
  map<string, bool> seen;
  unordered_map<string, int> distance;
  map<string, list<Edge>> adj;
  for (const auto &v : vertices) {
    adj[v.getID()] = v.getEdges();
    distance[v.getID()] = INT_MAX;
  }
  distance[startLabel] = 0;
  pq.push(make_pair(0, startLabel));

  while (!pq.empty()) {
    string u = pq.top().second;
    pq.pop();
    seen[u] = true;
    for (const auto &edge : adj[u]) {
      string v = edge.getDestination();
      int w = edge.getWeight();
      if (!seen[v] && distance[v] > distance[u] + w) {
        distance[v] = w;
        path[v] = u;
        pq.push(make_pair(distance[v], v));
      }
    }
  }
  distance.erase(startLabel);
  for (const auto &it : distance) {
    if (path[it.first] != it.first) {
      visit(path[it.first], it.first, it.second);
      result += it.second;
    }
  }
  return result;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }
  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i) {
    myfile >> fromVertex >> toVertex >> weight;
    add(fromVertex);
    add(toVertex);
    connect(fromVertex, toVertex, weight);
  }
  myfile.close();
  return true;
}

// Checks if edge exist between one string and another string.
bool Graph::checkIfEdgeExist(const string &from, const string &to) const {
  for (auto &v : vertices) {
    if (v.getID() == from) {
      for (const auto &edge : v.getEdges()) {
        if (edge.getDestination() == to) {
          return true;
        }
      }
    }
  }
  return false;
}
