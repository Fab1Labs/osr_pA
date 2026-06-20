#include "gtest/gtest.h"

#include <filesystem>
#include <iostream>

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "osr/cch_preprocessing.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, string_cache) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/map.osm", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  // 140186757 and 519430215 have the same name=Pankratiusstraße
  const auto pankratius1 = w.find_way(osm_way_idx_t{140186757});
  const auto pankratius2 = w.find_way(osm_way_idx_t{519430215});
  ASSERT_TRUE(pankratius1.has_value());
  ASSERT_TRUE(pankratius2.has_value());

  const auto name_idx1 = w.way_names_[pankratius1.value()];
  const auto name_idx2 = w.way_names_[pankratius2.value()];
  ASSERT_EQ(name_idx1, name_idx2);
}

TEST(extract, bus_only_on_highway) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/luisenplatz-darmstadt.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto const luisenplatz_outer = w.find_way(osm_way_idx_t{53341306});
  ASSERT_TRUE(luisenplatz_outer.has_value());

  auto const& wp = w.r_->way_properties_[luisenplatz_outer.value()];
  ASSERT_FALSE(wp.is_bus_accessible());
  ASSERT_TRUE(wp.is_foot_accessible());
}

TEST(extract, contraction_order) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto mip = cch::mip_proc{w};
  mip.build_contraction_order();

  bool eq = true;
  for (auto const [rank, node] : utl::enumerate(mip.contr_order_)) {
    eq = eq && (static_cast<std::uint32_t>(rank) == mip.ways_.r_->node_importance_[node]);
  }
  
  ASSERT_TRUE(eq);
}

TEST(extract, init_neighborhoods) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto mip = cch::mip_proc{w};
  mip.build_contraction_order();
  mip.init_neighborhoods();

  // Test empty neighborhood: Well at the "Aachen" name on the map
  ASSERT_EQ(mip.all_neighbors_[19851].node_, osr::node_idx_t{7884});
  ASSERT_TRUE(mip.all_neighbors_[19851].neighbors_.empty());

  // Test neighborhood with one higher neighbor: Aachen - Dennewartstrasse
  ASSERT_EQ(mip.all_neighbors_[1685].node_, osr::node_idx_t{17169});
  ASSERT_EQ(mip.all_neighbors_[1685].neighbors_.size(), 1);
  ASSERT_EQ(mip.all_neighbors_[1685].neighbors_[0], std::tuple(osr::node_idx_t{17170}, static_cast<std::uint32_t>(1686), false, osr::node_idx_t{17170}, osr::way_idx_t{13036}, osr::way_idx_t{13036}));

  // Test neighborhood with multiple neighbors: Aachen - Gabelung Büchel
  ASSERT_EQ(mip.all_neighbors_[14241].node_, osr::node_idx_t{14653});
  ASSERT_EQ(mip.all_neighbors_[14245].node_, osr::node_idx_t{2761});
  ASSERT_EQ(mip.all_neighbors_[14251].node_, osr::node_idx_t{201});
  ASSERT_EQ(mip.all_neighbors_[14283].node_, osr::node_idx_t{200});
  ASSERT_EQ(mip.all_neighbors_[14241].neighbors_[0], std::tuple(osr::node_idx_t{201}, static_cast<std::uint32_t>(14251), false, osr::node_idx_t{201}, osr::way_idx_t{6255}, osr::way_idx_t{6255}));
  ASSERT_EQ(mip.all_neighbors_[14241].neighbors_[1], std::tuple(osr::node_idx_t{2761}, static_cast<std::uint32_t>(14245), false, osr::node_idx_t{2761}, osr::way_idx_t{6255}, osr::way_idx_t{6255}));
  ASSERT_EQ(mip.all_neighbors_[14241].neighbors_[2], std::tuple(osr::node_idx_t{200}, static_cast<std::uint32_t>(14283), false, osr::node_idx_t{200}, osr::way_idx_t{10495}, osr::way_idx_t{10495}));

}

TEST(extract, sort_neighbors) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto ex1_n = cch::neighborhood{osr::node_idx_t{3}, static_cast<std::uint32_t>(23)};
  ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{1}, static_cast<std::uint32_t>(36), false, osr::node_idx_t{3}, osr::way_idx_t{1}, osr::way_idx_t{1}));
  ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{2}, static_cast<std::uint32_t>(28), false, osr::node_idx_t{3}, osr::way_idx_t{2}, osr::way_idx_t{2}));
  ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{4}, static_cast<std::uint32_t>(27), false, osr::node_idx_t{3}, osr::way_idx_t{4}, osr::way_idx_t{4}));
  ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{2}, static_cast<std::uint32_t>(28), false, osr::node_idx_t{3}, osr::way_idx_t{2}, osr::way_idx_t{2}));

  osr::vec<std::tuple<osr::node_idx_t, std::uint32_t, bool, osr::node_idx_t, osr::way_idx_t, osr::way_idx_t>> ex1_n_exp;
  ex1_n_exp.push_back(std::tuple(osr::node_idx_t{4}, static_cast<std::uint32_t>(27), false, osr::node_idx_t{3}, osr::way_idx_t{4}, osr::way_idx_t{4}));
  ex1_n_exp.push_back(std::tuple(osr::node_idx_t{2}, static_cast<std::uint32_t>(28), false, osr::node_idx_t{3}, osr::way_idx_t{2}, osr::way_idx_t{2}));
  ex1_n_exp.push_back(std::tuple(osr::node_idx_t{1}, static_cast<std::uint32_t>(36), false, osr::node_idx_t{3}, osr::way_idx_t{1}, osr::way_idx_t{1}));

  ex1_n.sort_neighbors();
  ASSERT_EQ(ex1_n.neighbors_, ex1_n_exp);
}

// TEST(extract, neighborhood_concat) {
//   auto p = fs::temp_directory_path() / "osr_test";
//   auto ec = std::error_code{};
//   fs::remove_all(p, ec);
//   fs::create_directories(p, ec);

//   extract(false, "test/aachen.osm.pbf", p, {});

//   auto ex1_n = cch::neighborhood(osr::node_idx_t{10}, static_cast<std::uint32_t>(13));
//   auto ex2_n = cch::neighborhood(osr::node_idx_t{11}, static_cast<std::uint32_t>(12));

//   ex2_n.concatenate(ex1_n.neighbors_);
//   ASSERT_TRUE(ex2_n.neighbors_.empty());

//   ex2_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
//   ex2_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(2)));

//   ex1_n.concatenate(ex2_n.neighbors_);
//   ASSERT_EQ(ex1_n.neighbors_[0], std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
//   ASSERT_TRUE(ex1_n.neighbors_.size() == 1);
// }


TEST(extract, elimination_tree) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto mip = cch::mip_proc{w};
  mip.build_contraction_order();
  mip.init_neighborhoods();
  mip.contract_nodes();

  // neighbor without any neighbors:
  ASSERT_EQ(mip.elimination_tree_[0], static_cast<std::uint32_t>(9663));
  ASSERT_EQ(mip.elimination_tree_[1], static_cast<std::uint32_t>(2));
}