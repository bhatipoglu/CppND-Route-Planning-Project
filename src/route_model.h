#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model
{

public:
  /**
   * @class Node
   * Represents a node in the route model.
   * Inherits from Model::Node.
   */
  class Node : public Model::Node
  {
  public:
    Node *parent = nullptr;                            /**< Pointer to the parent node. */
    float h_value = std::numeric_limits<float>::max(); /**< Heuristic value of the node. */
    float g_value = 0.0;                               /**< Cost from the start node to the current node. */
    bool visited = false;                              /**< Flag indicating if the node has been visited. */
    std::vector<Node *> neighbors;                     /**< Vector of neighboring nodes. */

    /**
     * Finds the neighboring nodes of the current node.
     */
    void FindNeighbors();

    /**
     * Calculates the Euclidean distance between the current node and another node.
     * @param other The other node to calculate the distance to.
     * @return The Euclidean distance between the two nodes.
     */
    float distance(Node other) const
    {
      return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
    }

    Node() {} /**< Default constructor. */
    /**
     * Constructor that initializes the node with an index, a search model, and a base node.
     * @param idx The index of the node.
     * @param search_model The search model.
     * @param node The base node.
     */
    Node(int idx, RouteModel *search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

  private:
    int index; /**< The index of the node. */
    /**
     * Finds a neighbor node based on a vector of node indices.
     * @param node_indices The vector of node indices.
     * @return A pointer to the neighbor node.
     */
    Node *FindNeighbor(std::vector<int> node_indices);
    RouteModel *parent_model = nullptr; /**< Pointer to the parent model. */
  };

  RouteModel(const std::vector<std::byte> &xml);
  Node &FindClosestNode(float x, float y);
  auto &SNodes() { return m_Nodes; }
  std::vector<Node> path;

private:
  void CreateNodeToRoadHashmap();
  std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
  std::vector<Node> m_Nodes;
};

#endif
