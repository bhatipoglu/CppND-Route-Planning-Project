#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


/**
 * @class RoutePlanner
 * @brief A class that represents a route planner for a given map.
 * 
 * The RoutePlanner class is responsible for finding the shortest path between two points on a map.
 * It uses the A* search algorithm to efficiently explore the map and find the optimal path.
 * 
 * The class provides public methods to initialize the route planner, perform the A* search, and retrieve the distance of the optimal path.
 * It also provides private methods and variables to assist in the search process.
 */
class RoutePlanner {
  public:
    /**
     * @brief Constructs a RoutePlanner object.
     * 
     * @param model The RouteModel object representing the map.
     * @param start_x The x-coordinate of the starting point.
     * @param start_y The y-coordinate of the starting point.
     * @param end_x The x-coordinate of the destination point.
     * @param end_y The y-coordinate of the destination point.
     */
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);

    /**
     * @brief Gets the distance of the optimal path.
     * 
     * @return The distance of the optimal path.
     */
    float GetDistance() const {return distance;}

    /**
     * @brief Performs the A* search algorithm to find the optimal path.
     */
    void AStarSearch();

  private:
    /**
     * @brief Adds neighboring nodes to the open list.
     * 
     * @param current_node The current node being explored.
     */
    void AddNeighbors(RouteModel::Node *current_node);

    /**
     * @brief Calculates the heuristic value (H value) for a given node.
     * 
     * @param node The node for which to calculate the H value.
     * @return The calculated H value.
     */
    float CalculateHValue(RouteModel::Node const *node);

    /**
     * @brief Constructs the final path from the start node to the end node.
     * 
     * @param end_node The end node of the optimal path.
     * @return A vector of nodes representing the optimal path.
     */
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *end_node);

    /**
     * @brief Finds the next node to explore from the open list.
     * 
     * @return A pointer to the next node to explore.
     */
    RouteModel::Node *NextNode();

    // Private variables
    std::vector<RouteModel::Node*> open_list; ///< The list of nodes to explore.
    RouteModel::Node *start_node; ///< The starting node.
    RouteModel::Node *end_node; ///< The destination node.
    float distance = 0.0f; ///< The distance of the optimal path.
    RouteModel &m_Model; ///< The RouteModel object representing the map.
};

#endif