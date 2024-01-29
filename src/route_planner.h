#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


/**
 * @class RoutePlanner
 * @brief Represents a route planner for finding the shortest path between two points on a map.
 *
 * The RoutePlanner class uses the A* search algorithm to find the shortest path between a start and end point
 * on a given map. It takes a RouteModel object, start and end coordinates as input, and provides methods to
 * calculate the distance, perform the A* search, add neighbors to a node, calculate the heuristic value,
 * construct the final path, and find the next node in the search.
 */
class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;}
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    float distance = 0.0f;
    RouteModel &m_Model;
};

#endif