#include <iostream>
#include <math.h>
#include <vector>
#include <iterator>
#include <fstream>
#include <queue>
#include "src/matplotlibcpp.h" //Graph Library

using namespace std;
namespace plt = matplotlibcpp;

// Map class
class Map {
public:
    const static int mapHeight = 300;
    const static int mapWidth = 150;
    vector<vector<double> > map = GetMap();
    vector<vector<int> > grid = MaptoGrid();
    vector<vector<int> > heuristic = GenerateHeuristic();

private:
    // Read the file and get the map
    vector<vector<double> > GetMap()
    {
        vector<vector<double> > mymap(mapHeight, vector<double>(mapWidth));
        ifstream myReadFile;
        myReadFile.open("map.txt");

        while (!myReadFile.eof()) {
            for (int i = 0; i < mapHeight; i++) {
                for (int j = 0; j < mapWidth; j++) {
                    myReadFile >> mymap[i][j];
                }
            }
        }
        return mymap;
    }

    //Convert the map to 1's and 0's
    vector<vector<int> > MaptoGrid()
    {
        vector<vector<int> > grid(mapHeight, vector<int>(mapWidth));
        for (int x = 0; x < mapHeight; x++) {
            for (int y = 0; y < mapWidth; y++) {
                grid[x][y] = map[x][y] >= 0;
            }
        }

        return grid;
    }

    // Generate a Manhattan Heuristic Vector
    vector<vector<int> > GenerateHeuristic()
    {
        vector<vector<int> > heuristic(mapHeight, vector<int>(mapWidth));
        int goal[2] = { 60, 50 };
        for (int i = 0; i < heuristic.size(); i++) {
            for (int j = 0; j < heuristic[0].size(); j++) {
                int xd = goal[0] - i;
                int yd = goal[1] - j;
                // Manhattan Distance
                   int d = abs(xd) + abs(yd);
                // Euclidian Distance
                // double d = sqrt(xd * xd + yd * yd);
                // Chebyshev distance
                // int d = max(abs(xd), abs(yd));
                heuristic[i][j] = d;
            }
        }
        return heuristic;
    }
};

// Planner class
class Planner : Map {
public:
    int start[2] = { 230, 145 };
    int goal[2] = { 60, 50 };
    int cost = 1;

    string movements_arrows[4] = { "^", "<", "v", ">" };

    vector<vector<int> > movements{
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };

    vector<vector<int> > path;
};

// Printing vectors of any type
template <typename T>
void print2DVector(T Vec)
{
    for (int i = 0; i < Vec.size(); ++i) {
        for (int j = 0; j < Vec[0].size(); ++j) {
            cout << Vec[i][j] << ' ';
        }
        cout << endl;
    }
}

struct Node {
    std::vector<int> triplet;
    int priority;
 
    bool operator<(const Node& other) const {
        return priority > other.priority;
    }
};

Planner search(Map map, Planner planner)
{
    std::vector<std::vector<int>> expansion_vector(map.mapHeight, std::vector<int>(map.mapWidth, -1));
    std::vector<std::vector<int>> decision_vector(map.mapHeight, std::vector<int>(map.mapWidth, -1));

    std::priority_queue<Node> nodes_queue;

    nodes_queue.push({{0, planner.start[0], planner.start[1]}, 
        map.heuristic[planner.start[0]][planner.start[1]]});
    
    int iter_cnt = 0;
    expansion_vector[planner.start[0]][planner.start[1]] = iter_cnt++;
    
    while (not nodes_queue.empty()) {
        auto node = nodes_queue.top();
        auto& current_node = node.triplet;
        nodes_queue.pop();
        
        for (size_t i = 0; i < 4; ++i) {
            const auto& movement = planner.movements[i];
            std::vector<int> neighbor_node {current_node[0] + planner.cost, 
                current_node[1] + movement[0],
                current_node[2] + movement[1]};
            
            // if goal reached
            if (neighbor_node[1] == planner.goal[0] and 
                neighbor_node[2] == planner.goal[1] ) {
                    
                expansion_vector[neighbor_node[1]][neighbor_node[2]] = iter_cnt++;
                decision_vector[neighbor_node[1]][neighbor_node[2]] = i;
                //print2DVector(expansion_vector);
                
                std::vector<std::vector<string>> policy(map.mapHeight, std::vector<string>(map.mapWidth, "-"));
                policy[planner.goal[0]][planner.goal[1]] = "*";;

                int x_end = planner.goal[0];
                int y_end = planner.goal[1];
                int decision = decision_vector[neighbor_node[1]][neighbor_node[2]];

                while (x_end != planner.start[0] and y_end != planner.start[1]) {
                    planner.path.push_back({x_end, y_end});
                    auto decision_move = planner.movements[decision];
                    x_end -= decision_move[0];
                    y_end -= decision_move[1];
                    //std::cout << x_end << " " << y_end << "\n";
                    policy[x_end][y_end] = planner.movements_arrows[decision];
                    decision = decision_vector[x_end][y_end]; 
                }

                //print2DVector(current_policy)
                return planner;
            }
            
            if (neighbor_node[1] >= 0 and neighbor_node[1] < map.mapHeight        // within map bounds
                and neighbor_node[2] >= 0 and neighbor_node[2] < map.mapWidth     // same
                and expansion_vector[neighbor_node[1]][neighbor_node[2]] == -1    // not yet visited
                and map.grid[neighbor_node[1]][neighbor_node[2]] == 0) {          // not an obstacle
                
                expansion_vector[neighbor_node[1]][neighbor_node[2]] = iter_cnt++;
                decision_vector[neighbor_node[1]][neighbor_node[2]] = i;
                
                auto new_priority = neighbor_node[0] + map.heuristic[neighbor_node[1]][neighbor_node[2]];
                nodes_queue.push({std::move(neighbor_node), new_priority});
            }
        }
    }

    std::cout << "Stuck\n";
    return planner;
}

void visualization(Map map, Planner planner)
{
    //Graph Format
    plt::title("Path");
    plt::xlim(0, map.mapHeight);
    plt::ylim(0, map.mapWidth);

    // Draw every grid of the map:
    for (double x = 0; x < map.mapHeight; x++) {
        cout << "Remaining Rows= " << map.mapHeight - x << endl;
        for (double y = 0; y < map.mapWidth; y++) {
            if (map.map[x][y] == 0) { //Green unkown state
                plt::plot({ x }, { y }, "g.");
            }
            else if (map.map[x][y] > 0) { //Black occupied state
                plt::plot({ x }, { y }, "k.");
            }
            else { //Red free state
                plt::plot({ x }, { y }, "r.");
            }
        }
    }

    // TODO: Plot start and end states in blue colors using o and * respectively
    plt::plot({static_cast<double>(planner.start[0])}, 
        {static_cast<double>(planner.start[1])}, "bo");

    plt::plot({static_cast<double>(planner.goal[0])}, 
        {static_cast<double>(planner.goal[1])}, "b*");
    
    // TODO: Plot the robot path in blue color using a .
    for (int i = 0; i < planner.path.size(); i++) {
        plt::plot({static_cast<double>(planner.path[i][0])}, 
        {static_cast<double>(planner.path[i][1])}, "b.");
    }

    std::cout << planner.path.size();
    
    //Save the image and close the plot
    plt::show();
    plt::save("./Images/Path.png");
    plt::clf();
}

int main()
{
    // Instantiate a planner and map objects
    Map map;
    Planner planner;
    // Generate the shortest Path using the Astar algorithm
    planner = search(map, planner);
    // Plot the Map and the path generated
    visualization(map, planner);

    return 0;
}

