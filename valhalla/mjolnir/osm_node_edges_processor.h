#pragma once

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace mjolnir {

// Structure holding an edge Id and forward flag
struct OSMIdsAndDirection {
  bool forward;
  baldr::EdgeInfo edge_info;
};

/**
 * Collects edges for ways in the graph.
 *
 * @param reader GraphReader to access graph data
 * @param filename File path for additional edge information
 * @return Map of edges with directions and associated OSM IDs
 */
void collect_edge_osm_ids(baldr::GraphReader& reader, const std::string& filename);

} // namespace mjolnir
} // namespace valhalla
