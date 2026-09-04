#include "gurka.h"
#include "baldr/graphconstants.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// Two independent corridors. "Twisty Ridge" zigzags between two rows of the grid;
// "Straight Flats" runs along a single row. They share no nodes on purpose -- if
// they shared endpoints the straight road would have to bend to reach them, which
// would give it curvature of its own and make the contrast meaningless.
//
// A 10 m grid keeps the zigzag's radius of curvature far below the ~250 m that
// baldr::kCurvyThreshold corresponds to, so the classification is not marginal.
constexpr double kGridSize = 10;

const std::string kMap = R"(
   B   D   F   H

 A   C   E   G   Z

 P   Q   R   S   T
)";

const gurka::ways kWays = {
    {"ABCDEFGHZ", {{"highway", "secondary"}, {"name", "Twisty Ridge"}}},
    {"PQRST", {{"highway", "secondary"}, {"name", "Straight Flats"}}},
};

gurka::map curvy_map;

} // namespace

class CurvyRoads : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kMap, kGridSize);
    curvy_map = gurka::buildtiles(layout, kWays, {}, {}, "test/data/gurka_curvy_roads");
  }
};

// The two corridors must actually sit on opposite sides of the threshold in the
// tiles, otherwise every assertion below would pass vacuously.
TEST_F(CurvyRoads, CorridorsStraddleTheThreshold) {
  baldr::GraphReader reader(curvy_map.config.get_child("mjolnir"));
  const auto twisty = std::get<1>(gurka::findEdge(reader, curvy_map.nodes, "Twisty Ridge", "Z"));
  const auto straight = std::get<1>(gurka::findEdge(reader, curvy_map.nodes, "Straight Flats", "T"));
  ASSERT_NE(twisty, nullptr);
  ASSERT_NE(straight, nullptr);
  EXPECT_GT(twisty->curvature(), baldr::kCurvyThreshold);
  EXPECT_LE(straight->curvature(), baldr::kCurvyThreshold);
}

// The regression that matters: the old frontend heuristic summed
// maneuver.road_class, a field the route response has never carried, so the
// reported figure was structurally always zero.
TEST_F(CurvyRoads, ReportsCurvyLengthOnCurvyRoute) {
  auto result = gurka::do_action(valhalla::Options::route, curvy_map, {"A", "Z"}, "auto",
                                 {{"/costing_options/auto/prefer_curvy_roads", "1"}});
  const auto& summary = result.directions().routes(0).legs(0).summary();
  EXPECT_TRUE(summary.has_curvy());
  EXPECT_GT(summary.curvy_length(), 0.0f);
  // Every curvy metre is also a routed metre.
  EXPECT_LE(summary.curvy_length(), summary.length() + 0.001f);
}

// A route that never leaves the straight corridor reports no curvy mileage.
TEST_F(CurvyRoads, ReportsNothingOnStraightRoute) {
  auto result = gurka::do_action(valhalla::Options::route, curvy_map, {"P", "T"}, "auto");
  const auto& summary = result.directions().routes(0).legs(0).summary();
  EXPECT_FALSE(summary.has_curvy());
  EXPECT_FLOAT_EQ(summary.curvy_length(), 0.0f);
}

// The measurement is a property of the geometry, not of the request: asking for
// curvy roads changes which edges get chosen, never how the chosen ones are counted.
TEST_F(CurvyRoads, CurvyLengthIsIndependentOfThePreference) {
  auto with = gurka::do_action(valhalla::Options::route, curvy_map, {"A", "Z"}, "auto",
                               {{"/costing_options/auto/prefer_curvy_roads", "1"}});
  auto without = gurka::do_action(valhalla::Options::route, curvy_map, {"A", "Z"}, "auto");
  EXPECT_FLOAT_EQ(with.directions().routes(0).legs(0).summary().curvy_length(),
                  without.directions().routes(0).legs(0).summary().curvy_length());
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
