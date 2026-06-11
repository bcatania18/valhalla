#include "mjolnir/osm_node_edges_processor.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphtile.h"

#include <fstream>

namespace valhalla {
namespace mjolnir {

namespace {

/**
 * Writes one CSV line containing:
 * edge_id,string edge id,graph_file,forward,osm_node_id,distance_to_next[,osm_node_id,distance_to_next,...]
 * @param edge_id The edge GraphId to write.
 * @param forward Whether the edge is traversed in its forward direction.
 * @param osm_ids The OSM node ids associated with the edge shape.
 * @param shape The edge shape points, aligned by index with osm_ids.
 * @param file The open output stream to append the CSV line to.
 */
void write_edge_osm_node(const baldr::GraphId edge_id, bool forward,
                          const std::vector<uint64_t>& osm_ids,
                          const std::vector<midgard::PointLL>& shape, std::ofstream& file) {

    // output the basics: edge_id,string edge id
    file << (uint64_t)edge_id
      << "," << edge_id
      << "," << baldr::GraphTile::FileSuffix(edge_id)
      << "," << (uint32_t)forward;

    // now for each osm id, output the distance between it and the next node
    auto elem_count = osm_ids.size();
    for (uint64_t i = 0; i < elem_count; ++i) {
      file << "," << osm_ids[i];
      if (i + 1 < elem_count) {
        file
          << "," << shape[i].Distance(shape[i + 1]);
      }
    }
    file << std::endl;
}
}

void collect_edge_osm_ids(baldr::GraphReader& reader, const std::string& filename) {
  std::ofstream edges_osm_nodes_file;
  edges_osm_nodes_file.open(filename, std::ofstream::out | std::ofstream::trunc);

  // Iterate through all tiles
  for (auto edge_id : reader.GetTileSet()) {
    // If tile does not exist, skip
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }

    // Trim reader if over-committed
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    baldr::graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const baldr::DirectedEdge* edge = tile->directededge(edge_id);

      // Skip transit, connection, and shortcut edges
      if (edge->IsTransitLine() || edge->use() == baldr::Use::kTransitConnection ||
          edge->use() == baldr::Use::kEgressConnection ||
          edge->use() == baldr::Use::kPlatformConnection || edge->is_shortcut()) {
        continue;
      }

      // Skip if the edge does not allow auto use
      if (!(edge->forwardaccess() & baldr::kAutoAccess)) {
        continue;
      }

      auto edge_info = tile->edgeinfo(edge);
      const auto& shape = edge_info.shape();
      const auto nodes = edge_info.osm_node_ids();

      if (shape.size() != nodes.size()) {
        LOG_ERROR("Edge shape and OSM node id count do not match " + std::to_string(edge_id));
      } else {
        // Extract the OSM node ids that make up each edge
        write_edge_osm_node(edge_id, edge->forward(), nodes, shape, edges_osm_nodes_file);
      }
    }
  }

  edges_osm_nodes_file.close();
}
} // namespace mjolnir
} // namespace valhalla
