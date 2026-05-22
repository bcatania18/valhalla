#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "mjolnir/osm_node_edges_processor.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdlib>
#include <filesystem>
#include <string>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
namespace vm = valhalla::mjolnir;

// Main application to create a list wayids and directed edges belonging
// to ways that are drivable.
int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "a program that creates a list of OSM nodes for each each auto-drivable valhalla edge.\n"
      "requires that map tiles have been built with mjolnir.keep_all_osm_node_ids set to true\n\n"
      );

    options.add_options()
      ("h,help", "Print this help message.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config))
      return EXIT_SUCCESS;

  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Create an unordered map of OSM ways Ids and their associated graph edges

  GraphReader reader(config.get_child("mjolnir"));

  std::filesystem::path file_path = {config.get<std::string>("mjolnir.tile_dir")};
  file_path.append("osm_node_edges.txt");

  // Collect all way edges
  vm::collect_edge_osm_ids(reader, file_path.string());

  LOG_INFO("Finished writing edges.");

  return EXIT_SUCCESS;
}
