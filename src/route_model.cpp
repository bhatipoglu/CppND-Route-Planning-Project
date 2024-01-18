#include "route_model.h"
#include <iostream>


/**
 * @brief Constructor for the RouteModel class.
 * 
 * This constructor initializes a RouteModel object using the provided XML data.
 * It creates RouteModel nodes based on the Model nodes and populates the m_Nodes vector.
 * It also calls the CreateNodeToRoadHashmap function to create a hashmap for efficient lookup of roads connected to each node.
 * 
 * @param xml The XML data used to initialize the RouteModel.
 */
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}


/**
 * @brief Creates a hashmap that maps each node to the closest road in the model.
 * 
 * This function creates a hashmap that associates each node in the model with the closest road. 
 * The hashmap is used to efficiently find the closest road to a given node during the A* search algorithm.
 */
void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}


/**
 * Finds a neighbor node based on the given node indices.
 * 
 * @param node_indices A vector of node indices to search for neighbors.
 * @return A pointer to the neighbor node if found, nullptr otherwise.
 */
RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}


/**
 * @brief Finds the neighboring nodes of the current node.
 * 
 * This function is used to find the neighboring nodes of the current node in the RouteModel.
 * Neighboring nodes are the nodes that are connected to the current node through edges.
 * 
 * @return void
 */
void RouteModel::Node::FindNeighbors() {
    for (auto & road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


/**
 * Finds the closest node in the RouteModel to the given coordinates (x, y).
 * 
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 * @return A reference to the closest node in the RouteModel.
 */
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                dist = input.distance(SNodes()[node_idx]);
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}