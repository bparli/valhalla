#include "gurka.h"
#include "baldr/graphconstants.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// Two ways from A to C. The short one runs straight through B, which carries a
// clearance posted on the NODE -- a bridge or tunnel portal, or a height_restrictor.
// The long way round is unrestricted.
//
// B is interior to "Underpass" and belongs to no other way, so it is not naturally an
// intersection. That is deliberate: the parser has to force a break at B for the
// restriction to have any edge to attach to, and if it does not, this map routes a
// 4 m vehicle under a 3 m bridge.
const std::string kMap = R"(
  A  B  C

  D     E
)";

const gurka::ways kWays = {
    {"ABC", {{"highway", "primary"}, {"name", "Underpass"}}},
    {"ADEC", {{"highway", "primary"}, {"name", "The Long Way"}}},
};

const gurka::nodes kNodes = {
    {"B", {{"maxheight", "3.0"}}},
};

gurka::map dim_map;

std::string route_name(const valhalla::Api& api) {
  std::string names;
  for (const auto& leg : api.directions().routes(0).legs()) {
    for (const auto& man : leg.maneuver()) {
      for (const auto& n : man.street_name()) {
        if (names.find(n.value()) == std::string::npos) {
          names += (names.empty() ? "" : "|") + n.value();
        }
      }
    }
  }
  return names;
}

} // namespace

class NodeDimensionRestrictions : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kMap, 100);
    dim_map = gurka::buildtiles(layout, kWays, kNodes, {},
                                "test/data/gurka_node_dimension_restrictions");
  }
};

// Without the fix there is nothing to find: the restriction is parsed onto no edge at all.
//
// It is attached to the edges LEAVING the restricted node, not those arriving at it.
// That covers both directions -- passing through B means taking an edge out of B whichever
// way you came -- while leaving the node itself reachable, so "route me to the underpass"
// still works for a vehicle that cannot fit under it.
TEST_F(NodeDimensionRestrictions, RestrictionReachesTheTile) {
  baldr::GraphReader reader(dim_map.config.get_child("mjolnir"));

  // The B->C edge: findEdge returns the edge ending at C, which begins at B.
  auto edge_id = std::get<0>(gurka::findEdge(reader, dim_map.nodes, "Underpass", "C"));
  ASSERT_TRUE(edge_id.is_valid()) << "no B->C edge -- the node did not force a break at B";

  auto tile = reader.GetGraphTile(edge_id);
  const auto* de = tile->directededge(edge_id);
  ASSERT_NE(de, nullptr);
  EXPECT_TRUE(de->access_restriction()) << "edge out of B carries no access restriction";

  bool found = false;
  for (const auto& r : tile->GetAccessRestrictions(edge_id.id(), baldr::kAllAccess)) {
    if (r.type() == baldr::AccessType::kMaxHeight) {
      found = true;
      EXPECT_EQ(r.value(), 300u) << "3.0 m should be stored as 300 cm";
      EXPECT_TRUE(r.modes() & baldr::kAutoAccess)
          << "must bind auto -- a motorhome is not a truck, and truck-only would be inert";
    }
  }
  EXPECT_TRUE(found) << "no kMaxHeight restriction on the edge out of B";
}

// The reverse direction is covered by the same attachment: coming from C, the B->A edge
// is the one that carries it.
TEST_F(NodeDimensionRestrictions, RestrictionAppliesInBothDirections) {
  baldr::GraphReader reader(dim_map.config.get_child("mjolnir"));
  auto edge_id = std::get<0>(gurka::findEdge(reader, dim_map.nodes, "Underpass", "A"));
  ASSERT_TRUE(edge_id.is_valid());
  auto tile = reader.GetGraphTile(edge_id);
  bool found = false;
  for (const auto& r : tile->GetAccessRestrictions(edge_id.id(), baldr::kAllAccess)) {
    found |= (r.type() == baldr::AccessType::kMaxHeight);
  }
  EXPECT_TRUE(found) << "B->A carries no restriction, so C->A would route under the bridge";
}

// And the same route driven the other way is detoured too.
TEST_F(NodeDimensionRestrictions, OverHeightVehicleIsDetouredInReverse) {
  auto result = gurka::do_action(valhalla::Options::route, dim_map, {"C", "A"}, "auto",
                                 {{"/costing_options/auto/height", "4.0"}});
  EXPECT_EQ(route_name(result).find("Underpass"), std::string::npos);
}

// A rig that fits goes the short way, which is also the control: it proves the detour
// below is caused by the height and not by the restriction breaking the road entirely.
TEST_F(NodeDimensionRestrictions, VehicleThatFitsTakesTheShortWay) {
  auto result = gurka::do_action(valhalla::Options::route, dim_map, {"A", "C"}, "auto",
                                 {{"/costing_options/auto/height", "2.5"}});
  EXPECT_NE(route_name(result).find("Underpass"), std::string::npos);
}

// The whole point: an over-height vehicle must not be routed under it.
TEST_F(NodeDimensionRestrictions, OverHeightVehicleIsDetoured) {
  auto result = gurka::do_action(valhalla::Options::route, dim_map, {"A", "C"}, "auto",
                                 {{"/costing_options/auto/height", "4.0"}});
  const auto names = route_name(result);
  EXPECT_NE(names.find("The Long Way"), std::string::npos) << "took: " << names;
  EXPECT_EQ(names.find("Underpass"), std::string::npos)
      << "routed a 4 m vehicle under a 3 m clearance: " << names;
}

// Height and width are independent; a wide-but-short vehicle is not affected by a
// height posting.
TEST_F(NodeDimensionRestrictions, WidthDoesNotTriggerAHeightRestriction) {
  auto result = gurka::do_action(valhalla::Options::route, dim_map, {"A", "C"}, "auto",
                                 {{"/costing_options/auto/width", "2.9"}});
  EXPECT_NE(route_name(result).find("Underpass"), std::string::npos);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
