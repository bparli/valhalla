#include "baldr/graphconstants.h"
#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

// US maxweight values must be converted from short tons to metric tonnes exactly ONCE.
//
// Two stages touch the number. lua/graph.lua (normalize_weight) reads the tag, and in US
// spellings -- pounds, "st", a bare number -- emits short tons. GraphEnhancer then
// converts every kMaxWeight in US, MM and LR tiles to metric, because only it knows the
// country. In Sept 2026 this fork made the Lua convert to metric too, so every US limit
// was converted twice and came out about 9% low: Holly Street in San Carlos, CA, posted
// "7 st", was stored as 5.76 t instead of 6.35 t. These tests pin the end-to-end value,
// so a change at either stage that doubles (or drops) the conversion fails here.
//
// 7 short tons = 6.350 t, stored in hundredths: 635.

namespace {

// A->D directly on three weight-limited blocks, each spelled a different US way, or the
// long way round with no limit at all.
const std::string kMap = R"(
  A--B--C--D
  |        |
  E--------F
)";

const gurka::ways kWays = {
    {"AB", {{"highway", "secondary"}, {"name", "Holly Street"}, {"maxweight", "7 st"}}},
    {"BC", {{"highway", "secondary"}, {"name", "Holly Street"}, {"maxweight", "14000 lbs"}}},
    {"CD", {{"highway", "secondary"}, {"name", "Holly Street"}, {"maxweight", "7"}}},
    {"AEFD", {{"highway", "secondary"}, {"name", "The Long Way"}}},
};

// Ohio, inside test/data/language_admin.sqlite (the origin test_admin.cc uses, west of
// the Pennsylvania line), so the tiles are tagged US and GraphEnhancer's conversion runs.
const midgard::PointLL kOhio{-80.527768076, 41.934220471};

uint32_t stored_maxweight(gurka::map& map, const std::string& from, const std::string& to) {
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto edge_id = std::get<0>(gurka::findEdgeByNodes(reader, map.nodes, from, to));
  EXPECT_TRUE(edge_id.is_valid()) << from << to;
  auto tile = reader.GetGraphTile(edge_id);
  for (const auto& r : tile->GetAccessRestrictions(edge_id.id(), baldr::kAllAccess)) {
    if (r.type() == baldr::AccessType::kMaxWeight) {
      return static_cast<uint32_t>(r.value());
    }
  }
  ADD_FAILURE() << "no kMaxWeight on " << from << to;
  return 0;
}

std::string route_names(const valhalla::Api& api) {
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

gurka::map us_map;
gurka::map no_country_map;

} // namespace

class USWeightUnits : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    us_map = gurka::buildtiles(gurka::detail::map_to_coordinates(kMap, 100, kOhio), kWays, {}, {},
                               "test/data/gurka_us_weight_units",
                               {{"mjolnir.admin",
                                 {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}}});
    no_country_map = gurka::buildtiles(gurka::detail::map_to_coordinates(kMap, 100), kWays, {},
                                       {}, "test/data/gurka_us_weight_units_no_country");
  }
};

// The Holly Street case: "7 st" is 6.35 t, not 5.76 t.
TEST_F(USWeightUnits, ShortTonsAreConvertedOnce) {
  EXPECT_EQ(stored_maxweight(us_map, "A", "B"), 635u)
      << "7 st must be stored as 6.35 t; 576 means short tons were converted twice";
}

// Pounds are 57.9% of US maxweight tags: 14,000 lb is the same 7 short tons.
TEST_F(USWeightUnits, PoundsAreConvertedOnce) {
  EXPECT_EQ(stored_maxweight(us_map, "B", "C"), 635u);
}

// A bare number on a US road is read the way US signs are posted, in short tons.
TEST_F(USWeightUnits, BareUSNumbersAreShortTons) {
  EXPECT_EQ(stored_maxweight(us_map, "C", "D"), 635u);
}

// Outside the US nothing converts, so the conversion seen above is GraphEnhancer's alone:
// the Lua passes 7 st, 14000 lbs and 7 through as the same 7.
TEST_F(USWeightUnits, NothingConvertsWithoutAUSCountry) {
  EXPECT_EQ(stored_maxweight(no_country_map, "A", "B"), 700u);
  EXPECT_EQ(stored_maxweight(no_country_map, "B", "C"), 700u);
  EXPECT_EQ(stored_maxweight(no_country_map, "C", "D"), 700u);
}

// What it means for routing. A 6.3 t vehicle is legal on a 7-short-ton street and must be
// sent down it; with the double conversion (5.76 t) it was detoured.
TEST_F(USWeightUnits, AVehicleUnderTheLimitTakesTheStreet) {
  auto result = gurka::do_action(valhalla::Options::route, us_map, {"A", "D"}, "auto",
                                 {{"/costing_options/auto/weight", "6.3"}});
  EXPECT_EQ(route_names(result), "Holly Street");
}

TEST_F(USWeightUnits, AVehicleOverTheLimitIsDetoured) {
  auto result = gurka::do_action(valhalla::Options::route, us_map, {"A", "D"}, "auto",
                                 {{"/costing_options/auto/weight", "6.4"}});
  EXPECT_EQ(route_names(result), "The Long Way");
}
