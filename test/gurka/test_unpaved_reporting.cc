#include "gurka.h"
#include "valhalla/worker.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// One straight corridor of four equal segments, so every length assertion is a
// simple multiple of the grid size and nothing depends on turn geometry.
//
//   A--B--C--D--E    paved, paved, gravel, gravel  -- arrives on gravel
//   F--G--H--I--J    paved, gravel, paved, paved   -- gravel in the middle
//
// A 100 m grid keeps the segments long enough that a kilometre reading is not
// dominated by rounding.
constexpr double kGridSize = 100;

const std::string kMap = R"(
   A  B  C  D  E

   F  G  H  I  J
)";

const gurka::ways kWays = {
    {"AB", {{"highway", "secondary"}, {"name", "Main"}, {"surface", "asphalt"}}},
    {"BC", {{"highway", "secondary"}, {"name", "Main"}, {"surface", "asphalt"}}},
    // The tail: the last two segments of A->E are gravel, so a trip that ends at
    // E arrives on unpaved and unpaved_tail_length should cover both of them.
    {"CD", {{"highway", "unclassified"}, {"name", "Gravel Run"}, {"surface", "gravel"}}},
    {"DE", {{"highway", "unclassified"}, {"name", "Gravel Run"}, {"surface", "gravel"}}},
    // Unpaved in the middle: F->J ends on pavement, so its tail is zero even
    // though it carries unpaved mileage.
    {"FG", {{"highway", "secondary"}, {"name", "Middle"}, {"surface", "asphalt"}}},
    {"GH", {{"highway", "unclassified"}, {"name", "Middle"}, {"surface", "gravel"}}},
    {"HI", {{"highway", "secondary"}, {"name", "Middle"}, {"surface", "asphalt"}}},
    {"IJ", {{"highway", "secondary"}, {"name", "Middle"}, {"surface", "asphalt"}}},
};

gurka::map unpaved_map;

// Nodes sit three grid columns apart in the map above, so each way is 300 m.
constexpr float kSegmentKm = 3 * kGridSize / 1000.0f;
constexpr float kTol = 0.02f;

} // namespace

class UnpavedReporting : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kMap, kGridSize);
    unpaved_map = gurka::buildtiles(layout, kWays, {}, {}, "test/data/gurka_unpaved_reporting");
  }
};

// The surfaces must actually land on opposite sides of DirectedEdge::unpaved()
// (surface >= kCompacted), otherwise every assertion below passes vacuously.
TEST_F(UnpavedReporting, SurfacesStraddleTheUnpavedPredicate) {
  baldr::GraphReader reader(unpaved_map.config.get_child("mjolnir"));
  const auto paved = std::get<1>(gurka::findEdge(reader, unpaved_map.nodes, "Main", "B"));
  const auto gravel = std::get<1>(gurka::findEdge(reader, unpaved_map.nodes, "Gravel Run", "D"));
  ASSERT_NE(paved, nullptr);
  ASSERT_NE(gravel, nullptr);
  EXPECT_FALSE(paved->unpaved());
  EXPECT_TRUE(gravel->unpaved());
}

// A route that arrives on gravel reports the mileage and reports that all of it
// is the run reaching the destination -- "the last N miles are unpaved".
TEST_F(UnpavedReporting, ReportsUnpavedTailOnArrival) {
  auto result = gurka::do_action(valhalla::Options::route, unpaved_map, {"A", "E"}, "auto");
  const auto& summary = result.directions().routes(0).legs(0).summary();
  EXPECT_TRUE(summary.has_unpaved());
  EXPECT_NEAR(summary.unpaved_length(), 2 * kSegmentKm, kTol);
  EXPECT_NEAR(summary.unpaved_tail_length(), 2 * kSegmentKm, kTol);
}

// Unpaved in the middle is still reported, but the tail is zero: the driver
// arrives on pavement, which is a different decision from arriving on gravel.
TEST_F(UnpavedReporting, ReportsNoTailWhenUnpavedIsMidRoute) {
  auto result = gurka::do_action(valhalla::Options::route, unpaved_map, {"F", "J"}, "auto");
  const auto& summary = result.directions().routes(0).legs(0).summary();
  EXPECT_TRUE(summary.has_unpaved());
  EXPECT_NEAR(summary.unpaved_length(), kSegmentKm, kTol);
  EXPECT_NEAR(summary.unpaved_tail_length(), 0.0f, kTol);
}

// A fully paved route reports has_unpaved=false rather than omitting the field:
// the client has to be able to tell "no unpaved" from "this binary does not
// report it", or an old binary reads as a clean all-clear.
TEST_F(UnpavedReporting, ReportsCleanOnPavedRoute) {
  auto result = gurka::do_action(valhalla::Options::route, unpaved_map, {"A", "C"}, "auto");
  const auto& summary = result.directions().routes(0).legs(0).summary();
  EXPECT_FALSE(summary.has_unpaved());
  EXPECT_NEAR(summary.unpaved_length(), 0.0f, kTol);
  EXPECT_NEAR(summary.unpaved_tail_length(), 0.0f, kTol);
}

// exclude_unpaved blocks *transitioning* onto unpaved rather than banning it, so
// a paved-only request that still has to finish on gravel is exactly the case
// that reporting-on-failure-only would miss. Whichever way the router resolves
// it, the two claims must agree: unpaved mileage is only ever reported for a
// route that contains some.
TEST_F(UnpavedReporting, ReportedMileageMatchesTheRouteUnderExcludeUnpaved) {
  try {
    auto result = gurka::do_action(valhalla::Options::route, unpaved_map, {"A", "E"}, "auto",
                                   {{"/costing_options/auto/exclude_unpaved", "1"}});
    const auto& summary = result.directions().routes(0).legs(0).summary();
    EXPECT_EQ(summary.has_unpaved(), summary.unpaved_length() > 0.0f);
    EXPECT_LE(summary.unpaved_length(), summary.length() + 0.001f);
    EXPECT_LE(summary.unpaved_tail_length(), summary.unpaved_length() + 0.001f);
  } catch (const valhalla_exception_t& e) {
    // No path is the other legitimate outcome, and it is the one the app's
    // retry-and-report path handles.
    EXPECT_EQ(e.code, 442);
  }
}

// The unpaved figures obey the requested units the same way scenic and curvy do.
TEST_F(UnpavedReporting, ConvertsToMiles) {
  auto result = gurka::do_action(valhalla::Options::route, unpaved_map, {"A", "E"}, "auto",
                                 {{"/directions_options/units", "miles"}});
  const auto& summary = result.directions().routes(0).legs(0).summary();
  EXPECT_NEAR(summary.unpaved_length(), 2 * kSegmentKm * midgard::kMilePerKm, kTol);
  EXPECT_NEAR(summary.unpaved_tail_length(), 2 * kSegmentKm * midgard::kMilePerKm, kTol);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
