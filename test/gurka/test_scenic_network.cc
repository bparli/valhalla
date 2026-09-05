#include "gurka.h"
#include "baldr/graphconstants.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// One road per network-tag shape we actually see in the curated scenic data.
// Upstream inspects net[2] only when the tag splits into exactly three tokens, so
// every other shape was silently not scenic -- 15 of 972 routes, since the feature
// shipped. Most were spur/alternate variants of byways whose main line is tagged
// through the map-match path, which is why nobody noticed.
const std::string kMap = R"(
  A----B----C----D----E----F----G----H
)";

const gurka::ways kWays = {
    {"AB", {{"highway", "secondary"}, {"name", "Three Token"}}},
    {"BC", {{"highway", "secondary"}, {"name", "Two Token"}}},
    {"CD", {{"highway", "secondary"}, {"name", "Four Token Alternate"}}},
    {"DE", {{"highway", "secondary"}, {"name", "Four Token Spur"}}},
    {"EF", {{"highway", "secondary"}, {"name", "County Road"}}},
    {"FG", {{"highway", "secondary"}, {"name", "Named Drive"}}},
    {"GH", {{"highway", "secondary"}, {"name", "Not Scenic"}}},
};

gurka::relations make_relations() {
  auto rel = [](const std::string& way, const std::string& network) {
    return gurka::relation{{{gurka::way_member, way, ""}},
                           {{"type", "route"}, {"route", "road"}, {"network", network}}};
  };
  return {
      rel("AB", "US:CA:Scenic"),                            // the shape that always worked
      rel("BC", "US:Scenic"),                               // two tokens
      rel("CD", "US:NY:Scenic:Alternate"),                  // four tokens
      rel("DE", "US:NJ:Scenic:Spur"),                       // four tokens
      rel("EF", "US:CA:CR:Scenic"),                         // four, scenic last
      rel("FG", "US:CA:San_Francisco:49_Mile_Scenic_Drive"), // names a road, not a designation
      rel("GH", "US:CA:Business"),                          // control
  };
}

gurka::map scenic_map;

bool edge_is_scenic(baldr::GraphReader& reader, const std::string& way,
                    const std::string& end_node) {
  const auto* de = std::get<1>(gurka::findEdge(reader, scenic_map.nodes, way, end_node));
  EXPECT_NE(de, nullptr) << way;
  return de != nullptr && de->scenic();
}

} // namespace

class ScenicNetwork : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kMap, 100);
    scenic_map = gurka::buildtiles(layout, kWays, {}, make_relations(),
                                   "test/data/gurka_scenic_network");
  }
};

// The case that has always worked. If this breaks, the whole scenic feature is broken
// and the rest of the file's failures would be noise.
TEST_F(ScenicNetwork, ThreeTokenStillWorks) {
  baldr::GraphReader reader(scenic_map.config.get_child("mjolnir"));
  EXPECT_TRUE(edge_is_scenic(reader, "Three Token", "B"));
}

// Everything below was missed before: the token count was wrong, so the tag was
// never even examined.
TEST_F(ScenicNetwork, TwoTokenIsScenic) {
  baldr::GraphReader reader(scenic_map.config.get_child("mjolnir"));
  EXPECT_TRUE(edge_is_scenic(reader, "Two Token", "C")) << "US:Scenic";
}

TEST_F(ScenicNetwork, FourTokenWithSuffixIsScenic) {
  baldr::GraphReader reader(scenic_map.config.get_child("mjolnir"));
  EXPECT_TRUE(edge_is_scenic(reader, "Four Token Alternate", "D")) << "US:NY:Scenic:Alternate";
  EXPECT_TRUE(edge_is_scenic(reader, "Four Token Spur", "E")) << "US:NJ:Scenic:Spur";
}

TEST_F(ScenicNetwork, ScenicInTheLastOfFourIsScenic) {
  baldr::GraphReader reader(scenic_map.config.get_child("mjolnir"));
  EXPECT_TRUE(edge_is_scenic(reader, "County Road", "F")) << "US:CA:CR:Scenic";
}

// Deliberately NOT scenic: the token names a road, it does not declare a designation.
// Substring matching would sweep this in along with anything merely named "scenic",
// so whole-token matching is the point rather than an accident.
TEST_F(ScenicNetwork, ARoadMerelyNamedScenicIsNotScenic) {
  baldr::GraphReader reader(scenic_map.config.get_child("mjolnir"));
  EXPECT_FALSE(edge_is_scenic(reader, "Named Drive", "G"))
      << "US:CA:San_Francisco:49_Mile_Scenic_Drive should not match on a substring";
}

TEST_F(ScenicNetwork, NonScenicNetworkIsNotScenic) {
  baldr::GraphReader reader(scenic_map.config.get_child("mjolnir"));
  EXPECT_FALSE(edge_is_scenic(reader, "Not Scenic", "H"));
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
