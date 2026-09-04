#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

#include <valhalla/baldr/conditional_speed_limit.h>
#include <valhalla/mjolnir/osmaccessrestriction.h>
#include <valhalla/mjolnir/osmlinguistic.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmrestriction.h>
#include <valhalla/mjolnir/uniquenames.h>

#include <cstdint>
#include <string>
#include <unordered_set>

namespace valhalla {
namespace mjolnir {

// OSM record type
enum class OSMType : uint8_t { kNode, kWay, kRelation };

// Structure to store OSM node information and associate it to an OSM way
struct OSMWayNode {
  OSMNode node;
  uint32_t way_index = 0;
  uint32_t way_shape_node_index = 0;
};

// Node coordinate for faster traversal where only shapes are needed.
struct OSMWayNodeShape {
  uint32_t lng7 = std::numeric_limits<uint32_t>::max();
  uint32_t lat7 = std::numeric_limits<uint32_t>::max();

  midgard::PointLL latlng() const {
    // if either coord is borked we return invalid ll
    if (lng7 == std::numeric_limits<uint32_t>::max() ||
        lat7 == std::numeric_limits<uint32_t>::max()) {
      return {};
    }
    return {lng7 * 1e-7 - 180, lat7 * 1e-7 - 90};
  }
};

// Structure to store OSM node information for BSS
struct OSMBSSNode {
  OSMNode node;
  // Index with serialized `BikeShareStationInfo` within the node_names list
  uint32_t bss_info_index;
};

// OSM bicycle data (stored within OSMData)
struct OSMBike {
  uint8_t bike_network;
  uint32_t name_index;
  uint32_t ref_index;
};

// OSM area data (stored within OSMData)
struct OSMAreaMember {
  uint64_t way_id;
  bool is_outer;
};

// OSM scenic route data (stored within OSMData)
struct OSMScenicRoute {
  uint32_t name_index;
  uint32_t ref_index;
  uint8_t tier; // Scenic bias tier (0=not scenic, 1=state, 2=national, 3=premier)
};

// OSM lane connectivity (stored within OSMData)
struct OSMLaneConnectivity {
  uint32_t to_way_id;
  uint32_t from_way_id;
  uint32_t to_lanes_index;   // Index to string in UniqueNames
  uint32_t from_lanes_index; // Index to string in UniqueNames
};

// Data types used within OSMData
using RestrictionsMultiMap = std::unordered_multimap<uint64_t, OSMRestriction>;
using ViaSet = std::unordered_set<uint64_t>;
using AccessRestrictionsMultiMap = std::unordered_multimap<uint64_t, OSMAccessRestriction>;
using BikeMultiMap = std::unordered_multimap<uint64_t, OSMBike>;
using AreaMultiMap = std::unordered_multimap<uint64_t, OSMAreaMember>;
using ScenicRouteMultiMap = std::unordered_multimap<uint64_t, OSMScenicRoute>;
using OSMLaneConnectivityMultiMap = std::unordered_multimap<uint64_t, OSMLaneConnectivity>;
using LinguisticMultiMap = std::unordered_multimap<uint64_t, OSMLinguistic>;
using ConditionalSpeedLimitsMultiMap =
    std::unordered_multimap<uint64_t, baldr::ConditionalSpeedLimit>;

// OSMString map uses the way Id as the key and the name index into UniqueNames as the value
using OSMStringMap = std::unordered_map<uint64_t, uint32_t>;

/**
 * Simple container for OSM data.
 * Populated by the PBF parser and sent into GraphBuilder.
 */
struct OSMData {
  /**
   * Write data to temporary files.
   * @return Returns true if successful, false if an error occurs.
   */
  bool write_to_temp_files(const std::string& tile_dir);

  /**
   * Read data from temporary files.
   * @return Returns true if successful, false if an error occurs.
   */
  bool read_from_temp_files(const std::string& tile_dir);

  /**
   * Read data from temporary unique name file.
   * @return Returns true if successful, false if an error occurs.
   */
  bool read_from_unique_names_file(const std::string& tile_dir);

  /**
   * add the direction information to the forward or reverse map for relations.
   */
  void add_to_name_map(const uint64_t member_id,
                       const std::string& direction,
                       const std::string& reference,
                       const bool forward = true);

  /**
   * Cleanup temporary files.
   */
  static void cleanup_temp_files(const std::string& tile_dir);

  uint64_t max_changeset_id_; // The largest/newest changeset id encountered when parsing OSM data
  uint64_t max_way_id = 0;  // Highest way id seen while parsing. Synthetic ids are assigned above it
  uint64_t max_node_id = 0; // Highest node id seen. Synthetic ids are assigned above it
  uint64_t osm_node_count;  // Count of osm nodes
  uint64_t osm_way_count;   // Count of osm ways
  uint64_t osm_way_node_count;    // Count of osm nodes on osm ways
  uint64_t node_count;            // Count of all nodes in the graph
  uint64_t edge_count;            // Estimated count of edges in the graph
  uint64_t node_ref_count;        // Number of node with ref
  uint64_t node_name_count;       // Number of nodes with names
  uint64_t node_exit_to_count;    // Number of nodes with exit_to
  uint64_t node_linguistic_count; // Number of nodes with linguistic info

  // D7 -- dimension-tag coverage counters, so a rebuild reports what it actually
  // ingested instead of us inferring coverage from taginfo afterwards. Default-
  // initialised because, unlike the counts above, they are written by only some code
  // paths. Reported by LogDimensionCoverage() at the end of parsing.
  // Only the node counters are stored: the node callback is the one place that sees a
  // dimension tag and decides to discard it. Way-level tags are filtered in lua before
  // C++ sees them, so a "way dropped" counter here would always read zero -- the way
  // side is instead summarised by walking the restriction maps in LogDimensionCoverage.
  uint64_t node_dimension_kept = 0;    // node maxheight/maxwidth that parsed to a usable value
  uint64_t node_dimension_dropped = 0; // present but empty or unparseable -- never guessed at

  // Stores simple restrictions. Indexed by the from way Id
  RestrictionsMultiMap restrictions;

  // unordered set used to find out if a wayid is included in any vias for complex restrictions
  ViaSet via_set;

  // Stores access restrictions. Indexed by the from way Id.
  AccessRestrictionsMultiMap access_restrictions;

  // Stores dimension restrictions posted on a NODE rather than a way -- bridge and
  // tunnel portals tagged on the point, and barrier=height_restrictor. Indexed by the
  // node Id, and applied in GraphBuilder to every edge incident to that node, since
  // reaching any of them means passing the restriction. Kept separate from
  // access_restrictions because that map is keyed by way and attaching a node's
  // clearance to a whole way would restrict miles of road on the strength of one point.
  // Small by construction: ~4,800 such nodes in the US extract.
  AccessRestrictionsMultiMap node_access_restrictions;

  // Stores bike information from the relations.  Indexed by the way Id.
  BikeMultiMap bike_relations;

  // Stores area information from the relations. Indexed by the relation Id.
  AreaMultiMap area_relations;

  // Stores scenic route information from the relations.  Indexed by the way Id.
  ScenicRouteMultiMap scenic_routes;

  // Map that stores an updated ref for a way. This needs to remain a map, since relations
  // update many ways at a time (so we can't move this into OSMWay unless that is mapped by Id).
  OSMStringMap way_ref;

  // Map that stores an updated reverse ref for a way. This needs to remain a map, since relations
  // update many ways at a time (so we can't move this into OSMWay unless that is mapped by Id).
  OSMStringMap way_ref_rev;

  // Unique names and strings for nodes. This is separate from other names/strings so that
  // the OSMNode (and OSMWayNode) structures can be made smaller.
  UniqueNames node_names;

  // Unique names and strings (includes road names, references, turn lane strings, etc.)
  UniqueNames name_offset_map;

  // Lane connectivity, index by the to way Id
  OSMLaneConnectivityMultiMap lane_connectivity_map;

  // Stores the pronunciations. Indexed by the way Id.
  LinguisticMultiMap pronunciations;

  // Stores the pronunciation languages. Indexed by the way Id.
  LinguisticMultiMap langs;

  // Stores the conditional speed limits ("maxspeed:conditional" osm key). Indexed by the way Id.
  ConditionalSpeedLimitsMultiMap conditional_speeds;

  bool initialized = false;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMDATA_H
