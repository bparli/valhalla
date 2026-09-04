#include "mjolnir/graph_lua_proc.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/osmdata.h"

#include <gtest/gtest.h>
#include <osmium/builder/osm_object_builder.hpp>

#include <string>

using namespace valhalla;

namespace {
struct TagsBuilder {
  osmium::memory::Buffer buffer;
  osmium::builder::WayBuilder way_builder;
  osmium::builder::TagListBuilder tags_builder;

  TagsBuilder()
      : buffer(1024, osmium::memory::Buffer::auto_grow::yes), way_builder(buffer),
        tags_builder(way_builder) {
  }

  void insert(const std::pair<std::string, std::string>& tag) {
    tags_builder.add_tag(tag);
  }

  const osmium::TagList& get() {
    return way_builder.object().tags();
  }
};

TEST(Lua, ZeroMantissa) {
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  TagsBuilder tags;
  // Check that decimals are properly parsed
  tags.insert({"highway", "primary"});
  tags.insert({"maxheight", "2.0"});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags.get());
  ASSERT_FLOAT_EQ(2.0f, std::stof(results["maxheight"]));
}

void assert_height_parses(const std::string& maxheight, float expected) {
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  TagsBuilder tags;
  tags.insert({"highway", "tertiary"});
  tags.insert({"maxheight", maxheight});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags.get());
  ASSERT_TRUE(results.count("maxheight") == 1);
  ASSERT_FLOAT_EQ(expected, std::stof(results["maxheight"]));
}

TEST(Lua, DefaultMeters) {
  // default for a number without units should be meters
  assert_height_parses("1.0", 1.0f);
  assert_height_parses("1", 1.0f);
  assert_height_parses("1.1", 1.1f);
}

TEST(Lua, ExplicitMeters) {
  // check all the common ways of writing meters
  assert_height_parses("1m", 1.0f);

  // there could be spaces, or not.
  assert_height_parses("1.1 m", 1.1f);
  assert_height_parses("1.1m", 1.1f);
  assert_height_parses("1 m", 1.0f);

  // but the units can change if they're plural
  assert_height_parses("1 meter", 1.0f);
  assert_height_parses("2meters", 2.0f);
}

TEST(Lua, UnitsCaseInsensitive) {
  // case doesn't matter for units
  assert_height_parses("1 METERS", 1.0f);
}

TEST(Lua, Centimeters) {
  // yes, this exists in the data...
  assert_height_parses("100cm", 1.0f);
}

TEST(Lua, EuropeanDecimal) {
  // it's common to see numbers written using the European convention of a
  // comma as the decimal separator.
  assert_height_parses("1,1", 1.1f);
}

TEST(Lua, FeetAndInches) {
  // there are a bunch of different commonly-used ways of writing heights in
  // feet and inches in OSM. here's a few, cropped from taginfo and generalised
  // to their generic formats.
  assert_height_parses("1ft1in", 0.33f);
  assert_height_parses("2 feet 1 inch", 0.64f);
  assert_height_parses("1 foot 6 inches", 0.46f);
  assert_height_parses("1.1 ft", 0.34f);
  assert_height_parses("1 ft", 0.3f);

  // feet and inches can also be written with single and double quotes (' and ")
  // and even with two single quotes (' and '').
  assert_height_parses("1'", 0.3f);
  assert_height_parses("1.1\"", 0.03f);
  assert_height_parses("1' 1\"", 0.33f);
  assert_height_parses("1'1\"", 0.33f);
  assert_height_parses("1'1''", 0.33f);
}

TEST(Lua, NumberDoublePeriod) {
  // Way 25494427 version 14 has a "maxheight" tag value of 3..35 with the two
  // dots. This probably shouldn't be parsed?
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  TagsBuilder tags;
  tags.insert({"highway", "tertiary"});
  tags.insert({"maxheight", "3..35"});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags.get());

  // check that the maxheight isn't present...
  ASSERT_TRUE(results.count("maxheight") == 0);

  // ... but that the results aren't completely empty
  ASSERT_TRUE(results.size() > 0);
}

void assert_weight_parses(const std::string& maxweight, float expected_tonnes) {
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  TagsBuilder tags;
  tags.insert({"highway", "primary"});
  tags.insert({"maxweight", maxweight});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags.get());
  ASSERT_TRUE(results.count("maxweight") == 1) << maxweight << " did not parse at all";
  EXPECT_NEAR(expected_tonnes, std::stof(results["maxweight"]), 0.01f) << "for " << maxweight;
}

// maxweight must come out in METRIC TONNES: AutoCost::ModeSpecificAllowed compares
// kMaxWeight against the request's weight in tonnes. Getting the unit wrong is
// permissive -- the router believes the bridge is stronger than the sign says --
// so these cases are safety-relevant, not cosmetic.
TEST(Lua, WeightMetricUnits) {
  assert_weight_parses("3.5", 3.5f); // OSM default unit is tonnes
  assert_weight_parses("3.5t", 3.5f);
  assert_weight_parses("3.5tonne", 3.5f);
  assert_weight_parses("3.5tonnes", 3.5f);
  assert_weight_parses("3500kg", 3.5f);
}

// The US-dominant spellings. "lbs" was previously divided by 2000 -- correct for
// short tons, but the result was then read as tonnes, so every value came out
// about 10% permissive. 57.9% of US maxweight tags are pounds.
TEST(Lua, WeightPounds) {
  assert_weight_parses("80000lbs", 36.29f);
  assert_weight_parses("80000lb", 36.29f);
  assert_weight_parses("2204.6226lbs", 1.0f);
}

// "st" and "lt" previously matched no branch at all and fell through to the bare
// number path, which reads as tonnes -- so "40st" became 40 t instead of 36.29 t.
// 34.4% of US maxweight tags are short tons.
TEST(Lua, WeightShortAndLongTons) {
  assert_weight_parses("40st", 36.29f);  // 40 short tons = 36.287 t
  assert_weight_parses("1st", 0.91f);
  assert_weight_parses("40lt", 40.64f);  // 40 long tons = 40.642 t
  assert_weight_parses("1lt", 1.02f);
}

// Bare "ton"/"tons" is ambiguous between short (US) and long (UK), so it is
// deliberately left as tonnes rather than guessed at. Pinned so the choice is
// visible if anyone revisits it.
TEST(Lua, WeightAmbiguousTonLeftAlone) {
  assert_weight_parses("10ton", 10.0f);
  assert_weight_parses("10tons", 10.0f);
}

TEST(Lua, TestForwardBackward) {
  // Way 25494427 version 14 has a "maxheight" tag value of 3..35 with the two
  // dots. This probably shouldn't be parsed?
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  TagsBuilder tags;
  tags.insert({"highway", "tertiary"});
  tags.insert({"maxheight:forward", "1"});
  tags.insert({"maxheight:backward", "1"});
  tags.insert({"maxlength:forward", "1"});
  tags.insert({"maxlength:backward", "1"});
  tags.insert({"maxwidth:forward", "1"});
  tags.insert({"maxwidth:backward", "1"});
  tags.insert({"maxweight:forward", "1"});
  tags.insert({"maxweight:backward", "1"});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags.get());

  // check that the maxheight is present...
  ASSERT_TRUE(results.count("maxheight_forward") == 1);
  ASSERT_TRUE(results.count("maxheight_backward") == 1);
  ASSERT_TRUE(results.count("maxlength_forward") == 1);
  ASSERT_TRUE(results.count("maxlength_backward") == 1);
  ASSERT_TRUE(results.count("maxwidth_forward") == 1);
  ASSERT_TRUE(results.count("maxwidth_backward") == 1);
  ASSERT_TRUE(results.count("maxweight_forward") == 1);
  ASSERT_TRUE(results.count("maxweight_backward") == 1);
}
} // namespace

// TODO: sweet jesus add more tests of this class!

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
