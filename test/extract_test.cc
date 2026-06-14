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

TEST(extract, neighborhood_initialisation) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto mip = cch::mip_proc{w};
  mip.build_contraction_order();
  mip.init_neighborhoods();
  
  // node with empty neighborhood:
  ASSERT_TRUE(mip.all_neighbors_[19851].neighbors_.empty());

  //nodes with one neighbor
  ASSERT_EQ(mip.all_neighbors_[2232].neighbors_[0].first, osr::node_idx_t{12958});
  ASSERT_EQ(mip.all_neighbors_[2232].neighbors_.size(), 1);

  ASSERT_EQ(mip.all_neighbors_[14331].neighbors_.size(), 1);
  ASSERT_EQ(mip.all_neighbors_[14331].neighbors_[0].first, osr::node_idx_t{17799});

  // bigger neighborhood
  osr::vec<std::pair<osr::node_idx_t, std::uint32_t>> ex1_neighbors_;
  ex1_neighbors_.push_back(std::pair(osr::node_idx_t{16699}, static_cast<std::uint32_t>(15387)));
  ex1_neighbors_.push_back(std::pair(osr::node_idx_t{14342}, static_cast<std::uint32_t>(15388)));
  //for (auto const& n : ex1_neighbors_) {ASSERT_EQ(std::count(mip.all_neighbors_[15383].neighbors_.begin(), mip.all_neighbors_[15383].neighbors_.end(), n), 1);}
  ASSERT_EQ(mip.all_neighbors_[15383].neighbors_, ex1_neighbors_);
}

TEST(extract, neighborhood_filter) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto ex1_n = cch::neighborhood{osr::node_idx_t{3}, static_cast<std::uint32_t>(23)};
  ex1_n.neighbors_.push_back(std::pair(osr::node_idx_t{1}, static_cast<std::uint32_t>(36)));
  ex1_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
  ex1_n.neighbors_.push_back(std::pair(osr::node_idx_t{4}, static_cast<std::uint32_t>(27)));
  ex1_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));

  osr::vec<std::pair<osr::node_idx_t, std::uint32_t>> ex1_n_exp;
  ex1_n_exp.push_back(std::pair(osr::node_idx_t{4}, static_cast<std::uint32_t>(27)));
  ex1_n_exp.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
  ex1_n_exp.push_back(std::pair(osr::node_idx_t{1}, static_cast<std::uint32_t>(36)));

  ex1_n.sort_neighbors();
  ASSERT_EQ(ex1_n.neighbors_, ex1_n_exp);
}

TEST(extract, neighborhood_concat) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto ex1_n = cch::neighborhood(osr::node_idx_t{10}, static_cast<std::uint32_t>(13));
  auto ex2_n = cch::neighborhood(osr::node_idx_t{11}, static_cast<std::uint32_t>(12));

  ex2_n.concatenate(ex1_n.neighbors_);
  ASSERT_TRUE(ex2_n.neighbors_.empty());

  ex2_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
  ex2_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(2)));

  ex1_n.concatenate(ex2_n.neighbors_);
  ASSERT_EQ(ex1_n.neighbors_[0], std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
  ASSERT_TRUE(ex1_n.neighbors_.size() == 1);
}
