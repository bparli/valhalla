#include "baldr/rapidjson_utils.h"
#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// Three disconnected corridors, each exercising one set of sign_points rules.
//
//   A..F  "Main": a node signal at C, a forward stop sign mid C-D, a forward
//         give-way mid D-E, posted limits on A-B (50) and C-D (30), unposted
//         B-C and D-E that fill from the same road, and E-F onto a different
//         road that must not borrow Main's limit.
//   P..S  "Long": posted 40 on P-Q, then 1.5 km unposted. Q-R fills (its posted
//         neighbour is 300 m back) but R-S does not, because the nearest posted
//         limit is beyond the 1 km fill cap.
//   J..L  An on-ramp with a ramp-meter signal mid-link and a copied motorway
//         maxspeed. Neither may surface: the meter would be pinned to the merge.
//   M..V  An off-ramp ending at a signalised intersection. That node signal is
//         the one that matters for guidance and must come through.
//   W..Z  A ramp meter mapped on the merge node X, where on-ramp Z-X joins the
//         I-3 mainline (US-101 N at Cypress Ave). Never a real signal, whether the
//         route passes along the mainline or comes up the ramp.
//   G..3  A freeway that ends at a real signal on a street (the Central Freeway at
//         Octavia): the route leaves I on a street, so the signal stands.
constexpr double kGridSize = 100;

const std::string kMap = R"(
          H
          |
A---B-----C---1---D---2---E---F

P--Q---------------R---S

J---5---K---L

M---N---T---U
        |
        V

W---X---Y
    |
    Z

G---I---O
    |
    3
)";

const gurka::ways kWays = {
    {"AB", {{"highway", "primary"}, {"name", "Main"}, {"maxspeed", "50"}}},
    {"BC", {{"highway", "primary"}, {"name", "Main"}}},
    {"C1D", {{"highway", "primary"}, {"name", "Main"}, {"maxspeed", "30"}}},
    {"D2E", {{"highway", "primary"}, {"name", "Main"}}},
    {"EF", {{"highway", "primary"}, {"name", "Other"}}},
    {"CH", {{"highway", "residential"}, {"name", "Side"}}},

    {"PQ", {{"highway", "primary"}, {"name", "Long"}, {"maxspeed", "40"}}},
    {"QR", {{"highway", "primary"}, {"name", "Long"}}},
    {"RS", {{"highway", "primary"}, {"name", "Long"}}},

    {"J5K",
     {{"highway", "motorway_link"}, {"oneway", "yes"}, {"name", "Onramp"}, {"maxspeed", "65 mph"}}},
    {"KL", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "I-1"}, {"maxspeed", "100"}}},

    {"MN", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "I-2"}, {"maxspeed", "100"}}},
    {"NT", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"name", "I-2"}}},
    {"TU", {{"highway", "primary"}, {"name", "Arterial"}}},
    {"TV", {{"highway", "primary"}, {"name", "Cross"}}},

    {"WX", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "I-3"}}},
    {"XY", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "I-3"}}},
    {"ZX", {{"highway", "motorway_link"}, {"oneway", "yes"}}},

    {"GI", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "Central Fwy"}}},
    {"IO", {{"highway", "primary"}, {"name", "Octavia"}}},
    {"I3", {{"highway", "primary"}, {"name", "Market"}}},
};

const gurka::nodes kNodes = {
    {"C", {{"highway", "traffic_signals"}}},
    {"1", {{"highway", "stop"}, {"direction", "forward"}}},
    {"2", {{"highway", "give_way"}, {"direction", "forward"}}},
    {"5", {{"highway", "traffic_signals"}}},
    {"T", {{"highway", "traffic_signals"}}},
    {"X", {{"highway", "traffic_signals"}}},
    {"I", {{"highway", "traffic_signals"}}},
};

gurka::map sign_map;

rapidjson::Document route_json(const std::vector<std::string>& waypoints,
                               const std::unordered_map<std::string, std::string>& options = {
                                   {"/sign_points", "1"}}) {
  std::string json;
  gurka::do_action(Options::route, sign_map, waypoints, "auto", options, {}, &json);
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  EXPECT_FALSE(doc.HasParseError());
  return doc;
}

// Every point must sit on the named node, and in the given order.
void expect_points(const rapidjson::Value& points, const std::vector<std::string>& expected) {
  ASSERT_TRUE(points.IsArray());
  ASSERT_EQ(points.Size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto& node = sign_map.nodes.at(expected[i]);
    EXPECT_NEAR(points[i][0].GetDouble(), node.lng(), 1e-6) << "point " << i << " " << expected[i];
    EXPECT_NEAR(points[i][1].GetDouble(), node.lat(), 1e-6) << "point " << i << " " << expected[i];
  }
}

// Speed-limit change points: (node, kph), kph 0 = the start of an unknown gap.
void expect_limits(const rapidjson::Value& limits,
                   const std::vector<std::pair<std::string, uint64_t>>& expected) {
  ASSERT_TRUE(limits.IsArray());
  ASSERT_EQ(limits.Size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto& node = sign_map.nodes.at(expected[i].first);
    EXPECT_NEAR(limits[i][0].GetDouble(), node.lng(), 1e-6) << "change " << i;
    EXPECT_NEAR(limits[i][1].GetDouble(), node.lat(), 1e-6) << "change " << i;
    EXPECT_EQ(limits[i][2].GetUint64(), expected[i].second) << "change " << i;
  }
}

} // namespace

class SignPoints : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kMap, kGridSize);
    sign_map = gurka::buildtiles(layout, kWays, kNodes, {}, "test/data/gurka_sign_points");
  }
};

// Older callers must see exactly the response they always did.
TEST_F(SignPoints, AbsentUnlessRequested) {
  auto doc = route_json({"A", "F"}, {});
  ASSERT_TRUE(doc.HasMember("trip"));
  EXPECT_FALSE(doc["trip"].HasMember("sign_points"));

  doc = route_json({"A", "F"}, {{"/sign_points", "0"}});
  EXPECT_FALSE(doc["trip"].HasMember("sign_points"));
}

TEST_F(SignPoints, SignsSitWhereTheDriverMeetsThem) {
  auto doc = route_json({"A", "F"});
  const auto& sp = doc["trip"]["sign_points"];
  expect_points(sp["traffic_signals"], {"C"});
  expect_points(sp["stop_signs"], {"D"});
  expect_points(sp["yield_signs"], {"E"});
}

// B-C and D-E fill from the same road; E-F is a different road, so it starts a gap.
TEST_F(SignPoints, SpeedLimitsFillWithinTheSameRoadOnly) {
  auto doc = route_json({"A", "F"});
  expect_limits(doc["trip"]["sign_points"]["speed_limits"], {{"A", 50}, {"C", 30}, {"E", 0}});
}

TEST_F(SignPoints, SpeedLimitFillStopsAtOneKilometre) {
  auto doc = route_json({"P", "S"});
  expect_limits(doc["trip"]["sign_points"]["speed_limits"], {{"P", 40}, {"R", 0}});
}

// The meter's edge flag would land on the merge, and the ramp's copied 65 mph is
// not trusted, so the first limit shown is the motorway's own.
TEST_F(SignPoints, OnRampMeterAndLimitAreIgnored) {
  auto doc = route_json({"J", "L"});
  const auto& sp = doc["trip"]["sign_points"];
  expect_points(sp["traffic_signals"], {});
  expect_limits(sp["speed_limits"], {{"K", 100}});
}

// The off-ramp shares the motorway's name, but a link is never filled.
TEST_F(SignPoints, OffRampKeepsItsNodeSignal) {
  auto doc = route_json({"M", "U"});
  const auto& sp = doc["trip"]["sign_points"];
  expect_points(sp["traffic_signals"], {"T"});
  expect_limits(sp["speed_limits"], {{"M", 100}, {"N", 0}});
}

// Multi-leg trips report one continuous set at the trip level.
TEST_F(SignPoints, MultiLegTripIsOneSequence) {
  auto doc = route_json({"A", "D", "F"});
  const auto& sp = doc["trip"]["sign_points"];
  expect_points(sp["traffic_signals"], {"C"});
  expect_points(sp["stop_signs"], {"D"});
  expect_points(sp["yield_signs"], {"E"});
}

// A signal-tagged merge node on a motorway is a mis-mapped ramp meter: drawn, it
// sits on the freeway mainline.
TEST_F(SignPoints, MergeNodeSignalOnAMotorwayIsDropped) {
  auto doc = route_json({"W", "Y"});
  expect_points(doc["trip"]["sign_points"]["traffic_signals"], {});

  doc = route_json({"Z", "Y"});
  expect_points(doc["trip"]["sign_points"]["traffic_signals"], {});
}

TEST_F(SignPoints, FreewayEndingAtASignalledStreetKeepsIt) {
  auto doc = route_json({"G", "O"});
  expect_points(doc["trip"]["sign_points"]["traffic_signals"], {"I"});
}
