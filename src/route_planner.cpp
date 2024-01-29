#include "route_planner.h"
#include <algorithm>

/**
 * @brief Constructs a RoutePlanner object.
 * 
 * @param model The RouteModel object representing the map.
 * @param start_x The x-coordinate of the starting point.
 * @param start_y The y-coordinate of the starting point.
 * @param end_x The x-coordinate of the ending point.
 * @param end_y The y-coordinate of the ending point.
 */
/**
 * @brief Constructs a RoutePlanner object.
 * 
 * This constructor initializes a RoutePlanner object with the given parameters.
 * It converts the input coordinates to percentages and uses the m_Model.FindClosestNode method
 * to find the closest nodes to the starting and ending coordinates. The found nodes are stored
 * in the RoutePlanner's start_node and end_node attributes.
 * 
 * @param model The RouteModel object.
 * @param start_x The x-coordinate of the starting point.
 * @param start_y The y-coordinate of the starting point.
 * @param end_x The x-coordinate of the ending point.
 * @param end_y The y-coordinate of the ending point.
 */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    // Find the closest nodes to the starting and ending coordinates
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

/**
 * Calculates the heuristic value (H value) for a given node.
 * The H value is the straight-line distance between the given node and the end node.
 *
 * @param node A pointer to the node for which the H value needs to be calculated.
 * @return The calculated H value.
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


/**
 * Adds neighboring nodes to the open list and updates their attributes.
 * 
 * @param current_node A pointer to the current node.
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate neighbors vector for the current node
    current_node->FindNeighbors();

    for(auto neighbor : current_node->neighbors) {
        // Set the parent node
        neighbor->parent = current_node;

        // Set the h_value using CalculateHValue method
        neighbor->h_value = CalculateHValue(neighbor);

        // Set the g_value to be the g_value of the current node plus the distance from the current node to the neighbor
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

        // Add the neighbor to open_list
        open_list.push_back(neighbor);

        // Set the node's visited attribute to true
        neighbor->visited = true;
    }
}


/**
 * @brief Returns the next node in the route.
 * 
 * This function is used to retrieve the next node in the route that needs to be visited.
 * 
 * @return A pointer to the next node in the route.
 */
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value
    std::sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b) {
        return (a->h_value + a->g_value) < (b->h_value + b->g_value);
    });

    // Create a pointer to the node in the list with the lowest sum
    RouteModel::Node *lowest_sum_node = open_list.front();

    // Remove that node from the open_list
    open_list.erase(open_list.begin());

    // Return the pointer
    return lowest_sum_node;
}

/**
 * @brief Represents a vector of RouteModel::Node objects.
 * 
 * This vector is used to store the nodes that make up the final path in the route planner.
 * Each element in the vector represents a node in the path.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // For each node in the chain, add the distance from the node to its parent to the distance variable
    while(current_node != start_node) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    // Add the start node to the path_found vector
    path_found.push_back(*start_node);

    // Reverse the path_found vector
    std::reverse(path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters
    distance *= m_Model.MetricScale();

    // Return the path_found vector
    return path_found;
}

/**
 * Performs the A* search algorithm to find the shortest path from the start node to the end node.
 */
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Set the start node's visited attribute to true
    start_node->visited = true;

    // Add the start node to the open_list
    open_list.push_back(start_node);

    while (!open_list.empty()) {
        // Get the next node from the open_list
        current_node = NextNode();

        // Check if the current node is the end node
        if (current_node == end_node) {
            // Construct the final path
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // Add all of the neighbors of the current node to the open_list
        AddNeighbors(current_node);
    }
}