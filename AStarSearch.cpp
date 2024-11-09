// Arnik Shah
// Robot Path Planning Project
// November 9, 2024

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <map>
#include <utility>
#include <iomanip> // For formatting output

using namespace std;

// Global Constants 
const int rows = 30;
const int cols = 50;
const vector<pair<int, int>> directions = {
            {1, 0},    // Right (0 degrees)
            {1, 1},    // Up-Right (45 degrees)
            {0, 1},    // Up (90 degrees)
            {-1, 1},   // Up-Left (135 degrees)
            {-1, 0},   // Left (180 degrees)
            {-1, -1},  // Down-Left (225 degrees)
            {0, -1},   // Down (270 degrees)
            {1, -1},   // Down-Right (315 degrees)
        };
const vector<int> angles = {0, 45, 90, 135, 180, 225, 270, 315};

// Function Prototype for calculating Euclidean distance
double euclideanDistance(pair<int, int> a, pair<int, int> b);

// Class to calculate path cost (f(n) values) for each node 
class NodePathCost {
public:
    NodePathCost(int k, int g) : k(k), g(g), h(0) {}

    void updateCost(int currentAngle, int newAngle, int action) {
        // Calculate Angle cost
        int angleDiff = abs(newAngle - currentAngle);
        if (angleDiff > 180) {
            angleDiff = 360 - angleDiff;
        }
        double angleCost = k * ((double)angleDiff / 180);
        // Calculate Distance cost
        double distanceCost = (action % 2 == 0) ? 1 : sqrt(2);
        // Update g(n) value
        g += angleCost + distanceCost;
    }

    void setHeuristicCost(pair<int, int> currPos, pair<int, int> goalPos) {
        // Get Euclidian distance between the current position and the goal position
        h = euclideanDistance(currPos, goalPos);
    }

    double getTotalCost() const { return g + h; }

    double getG() const { return g; }

private:
    int k;
    double g;
    double h;
};

// Class for each state
class Node {
public:
    Node(pair<int, int> loc, int angle, int k, int g) : location(loc), angle(angle), cost(k, g) {}

    void updateNodePathCost(int action, int newAngle, pair<int, int> goalPos) {
        cost.updateCost(angle, newAngle, action);
        cost.setHeuristicCost(location, goalPos);
        angle = newAngle;
    }

    double getCost() const { return cost.getTotalCost(); }

    double getTraveledcost() const { return cost.getG(); }

    pair<int, int> getLocation() const { return location; }

    int getAngle() const { return angle; }

    const vector<int>& getPrevActions() const { return prevActions; }

    const vector<double>& getPrevCosts() const { return prevfValues; }

    void setPrevActions(const vector<int>& actions) { prevActions = actions; }

    void setPrevCosts(const vector<double>& costs) { prevfValues = costs; }

    void addActions(int action) { prevActions.push_back(action); }

    void addCosts(double cost) { prevfValues.push_back(cost); }

    // Used to compare nodes in the frontier
    bool operator<(const Node& other) const { return this->getCost() > other.getCost(); }

    bool operator==(const Node& other) const { return location == other.location; }
private:
    pair<int, int> location;
    int angle;
    NodePathCost cost;
    // Stores the path actions taken to reach this node
    vector<int> prevActions;
    // Stores f(n) values along the path    
    vector<double> prevfValues;      
};

// A* Search Class
class AStarSearch {
public:
    AStarSearch(pair<int, int> start, pair<int, int> goal, int** workspace, int k)
        : start(start), goal(goal), workspace(workspace), k(k) {}

    ~AStarSearch() {
        // Free each row
        for (int c = 0; c < cols; ++c) {
            delete[] workspace[c]; 
        }
        // Free array of pointers
        delete[] workspace; 
    }

    bool solve() {
        priority_queue<Node> frontier;
        map<pair<int, int>, double> visited;
        // Add initial state to frontier and set
        Node startNode(start, 0, k, 0);
        frontier.push(startNode);
        visited[start] = startNode.getCost();

        while (!frontier.empty()) {
            Node current = frontier.top();
            frontier.pop();

            // Check if the current node is the goal
            if (current.getLocation() == goal) {
                solutionActions = current.getPrevActions();
                solutionFValues = current.getPrevCosts();
                solutionFValues.push_back(current.getCost()); // Include f(n) at the goal
                return true;
            }
            // Expand Node if current is not the goal
            // Check all possible actions
            for (size_t i = 0; i < directions.size(); ++i) {
                pair<int, int> newLoc = {
                    current.getLocation().first + directions[i].first,
                    current.getLocation().second + directions[i].second
                };
                // Check if location is in bounds and if it does not have an obstacle
                if (inBounds(newLoc) && workspace[newLoc.first][newLoc.second] != 1) {
                    // Make new Node and update its f(n) value
                    Node newNode(newLoc, current.getAngle(), k, current.getTraveledcost());
                    newNode.updateNodePathCost(i, angles[i], goal);

                    // Copy prev actions and prev f(n) values from the current node
                    newNode.setPrevActions(current.getPrevActions());
                    newNode.setPrevCosts(current.getPrevCosts());
                    newNode.addActions(i);
                    newNode.addCosts(newNode.getCost());
                    // Check if the new node is not in the map or has a smaller cost than one in the map
                    if (visited.find(newLoc) == visited.end() || newNode.getCost() < visited[newLoc]) {
                        frontier.push(newNode);
                        visited[newLoc] = newNode.getCost();
                        nodesGenerated++;
                    }
                }
            }
        }
        return false;
    }

    const vector<int>& getSolutionActions() const { return solutionActions; }

    const vector<double>& getSolutionFValues() const { return solutionFValues; }

    int getNodesGenerated() const { return nodesGenerated; }

private:
    pair<int, int> start, goal;
    int** workspace;
    int k;
    int nodesGenerated = 1;

    vector<int> solutionActions;
    vector<double> solutionFValues;

    bool inBounds(pair<int, int> loc) const {
        return loc.first >= 0 && loc.first < cols && loc.second >= 0 && loc.second < rows;
    }
};

// Function prototypes
pair<int, int> getPosition(ifstream& input);
int** getWorkspace(ifstream& input);
void outputResults(const string& filename, AStarSearch& search, int** currentState, pair<int, int> start);

int main() {
    ifstream input("Input1.txt");
    if (!input) {
        cerr << "Could not open the file.\n";
        exit(1);
    }

    pair<int, int> start = getPosition(input);
    pair<int, int> goal = getPosition(input);
    int** workspace = getWorkspace(input); 
    input.close();

    // Get k value
    int k;
    cout << "What is the value of k?" << endl;
    cin >> k;
    AStarSearch search(start, goal, workspace, k); 
    bool findSoultion = search.solve();

    // Output the nodes generated, solution actions, solution costs, and solution workspace
    if (findSoultion) {
        outputResults("file.txt", search, workspace, start); 
        cout << "Solution found and saved to output file." << endl;
    } else {
        cout << "No solution found." << endl;
    }
}

double euclideanDistance(pair<int, int> a, pair<int, int> b) {
    return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}

pair<int, int> getPosition(ifstream& input) {
    // Get starting and final coordinates
    int iCoordinate, jCoordinate = 0;
    input >> iCoordinate >> jCoordinate;
    return {iCoordinate, jCoordinate};
}

int** getWorkspace(ifstream& input) {
    // Allocate 50 columns
    int** matrix = new int*[cols];   
    for (int c = 0; c < cols; ++c) {
        // For each column, allocate 30 rows
        matrix[c] = new int[rows](); 
    }

    // Read the workspace values
    for (int j = rows - 1; j >= 0 ; --j) { 
        for (int i = 0; i < cols; ++i) { 
            input >> matrix[i][j]; 
        }
    }
    return matrix; // Return the raw pointer
}


void outputResults(const string& filename, AStarSearch& search, int** workspace, pair<int, int> start) {
    ofstream output(filename);
    // Depth of Node
    output << search.getSolutionActions().size() << endl; 
    // Total nodes generated
    output << search.getNodesGenerated() << endl;         

    // Solution path (actions)
    for (int action : search.getSolutionActions()) {
        output << action << " ";
    }
    output << endl;

    // f(n) values along the solution path
    for (double fValue : search.getSolutionFValues()) {
        // Show all values with 2 decimal points
        output << fixed << setprecision(2) << fValue << " ";
    }
    output << endl;

    // Mark the path in the currentState
    for (size_t i = 1; i < search.getSolutionActions().size(); ++i) {
        int action = search.getSolutionActions()[i];
        start.first += directions[action].first;
        start.second += directions[action].second;
        workspace[start.first][start.second] = 4;
    }

    // Print Workspace
    for (int row = rows - 1; row >= 0; --row) {
        for (int col = 0; col < cols; ++col) {
            output << workspace[col][row] << " ";
        }
        output << endl;
    }

    output.close();
}