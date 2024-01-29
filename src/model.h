#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <cstddef>

/**
 * @brief Represents a model of a map.
 * 
 * The Model class stores information about the nodes, ways, roads, railways, buildings, leisures, waters, and landuses in the map.
 * It provides methods to access and manipulate this information.
 */
class Model
{
public:
    struct Node {
        double x = 0.f; /**< The x-coordinate of the node. */
        double y = 0.f; /**< The y-coordinate of the node. */
    };
    
    struct Way {
        std::vector<int> nodes; /**< The list of node IDs that make up the way. */
    };
    
    struct Road {
        enum Type { Invalid, Unclassified, Service, Residential,
            Tertiary, Secondary, Primary, Trunk, Motorway, Footway }; /**< The type of the road. */
        int way; /**< The ID of the way that represents the road. */
        Type type; /**< The type of the road. */
    };
    
    struct Railway {
        int way; /**< The ID of the way that represents the railway. */
    };    

    struct Multipolygon {
        std::vector<int> outer; /**< The list of node IDs that make up the outer ring of the multipolygon. */
        std::vector<int> inner; /**< The list of node IDs that make up the inner rings of the multipolygon. */
    };

    struct Building : Multipolygon {};

    struct Leisure : Multipolygon {};

    struct Water : Multipolygon {};

    struct Landuse : Multipolygon {
        enum Type { Invalid, Commercial, Construction, Grass, Forest, Industrial, Railway, Residential }; /**< The type of the landuse area. */
        Type type; /**< The type of the landuse area. */
    };

    Model( const std::vector<std::byte> &xml );
 
    auto MetricScale() const noexcept { return m_MetricScale; }    

    auto &Nodes() const noexcept { return m_Nodes; }
    
    auto &Ways() const noexcept { return m_Ways; }
    
    auto &Roads() const noexcept { return m_Roads; }
    
    auto &Buildings() const noexcept { return m_Buildings; }
    
    auto &Leisures() const noexcept { return m_Leisures; }
    
    auto &Waters() const noexcept { return m_Waters; }

    auto &Landuses() const noexcept { return m_Landuses; }
    
    auto &Railways() const noexcept { return m_Railways; }
    
private:
    void AdjustCoordinates();
    
    /**
     * @brief Builds the rings of a multipolygon.
     * 
     * @param mp The multipolygon to build the rings for.
     */
    void BuildRings( Multipolygon &mp );
    
    /**
     * @brief Loads the map data from XML.
     * 
     * @param xml The XML data representing the map.
     */
    void LoadData(const std::vector<std::byte> &xml);
    
    std::vector<Node> m_Nodes; /**< The list of nodes in the map. */
    std::vector<Way> m_Ways; /**< The list of ways in the map. */
    std::vector<Road> m_Roads; /**< The list of roads in the map. */
    std::vector<Railway> m_Railways; /**< The list of railways in the map. */
    std::vector<Building> m_Buildings; /**< The list of buildings in the map. */
    std::vector<Leisure> m_Leisures; /**< The list of leisure areas in the map. */
    std::vector<Water> m_Waters; /**< The list of water areas in the map. */
    std::vector<Landuse> m_Landuses; /**< The list of landuse areas in the map. */
    
    double m_MinLat = 0.;
    double m_MaxLat = 0.;
    double m_MinLon = 0.;
    double m_MaxLon = 0.;
    double m_MetricScale = 1.f;
};
