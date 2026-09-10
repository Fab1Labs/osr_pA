#include "gtest/gtest.h"

#include <filesystem>
#include <iostream>
#include <algorithm>

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/routing/route.h"
#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "osr/cch_preprocessing.h"
#include "osr/cch_customization.h"
#include "osr/shortcut.h"

namespace fs = std::filesystem;
using namespace osr;

bool test_neighbors(osr::vec<cch::neighbor> const& neighbors,
                    osr::neighbor_idx_t const& probe,
                    osr::node_idx_t const& node,
                    std::uint32_t const& rank,
                    osr::way_idx_t const& way) {
  auto const result =  neighbors[probe].neighbor_ == node &&
                       neighbors[probe].rank_ == rank &&
                       neighbors[probe].edge_ == way;
  if (!result) {
    std::cout << "Exptected neighbor: " << neighbors[probe].neighbor_ << " but got: " << node << "\n";
    std::cout << "Exptected rank: " << neighbors[probe].rank_ << " but got: " << rank << "\n";
    std::cout << "Exptected way: " << neighbors[probe].edge_ << " but got: " << way << "\n";
  }

  return result;
}

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

TEST(extract, test_packed_and_unpacked_costs) {
  auto const data_dir = "test/aachen.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {});
  auto w = ways{p, cista::mmap::protection::READ};

  for (auto const [rank, node] : utl::enumerate(w.r_->contraction_order_)) {
    for (auto const [t_idx, target] : utl::enumerate(w.r_->sc_targets_[rank])) {
      auto const& shortcut = w.r_->cch_sc_up_[rank][t_idx];
      auto const& expected_cost = w.r_->cch_cost_up_[rank][t_idx];
      if (expected_cost == kInfeasible) {
        continue;
      }

      auto const path_up = w.r_->unpack_shortcut<direction::kForward, true>(shortcut);
      auto single_costs = w.r_->get_edge_cost<true>(shortcut.entry_node_.n_, shortcut.entry_node_.way_, 
                                                shortcut.entry_node_.dir_, path_up[0].n_, cost_t{120U});

      for(std::size_t i = 1; i < path_up.size(); ++i) {
        single_costs += w.r_->get_edge_cost<true>(path_up[i - 1].n_, path_up[i - 1].way_, path_up[i - 1].dir_, 
                                                     path_up[i].n_, cost_t{120U});
      }
      ASSERT_EQ(single_costs, expected_cost);
    }

    for (auto const [t_idx, target] : utl::enumerate(w.r_->sc_targets_[rank])) {
      auto const& shortcut = w.r_->cch_sc_down_[rank][t_idx];
      auto const& expected_cost = w.r_->cch_cost_down_[rank][t_idx];
      if (expected_cost == kInfeasible) {
        continue;
      }
      
      auto path_down = w.r_->unpack_shortcut<direction::kBackward, false>(shortcut);
      std::reverse(path_down.begin(), path_down.end());
      auto single_costs = w.r_->get_edge_cost<false>(shortcut.exit_node_.n_, shortcut.exit_node_.way_, 
                                                shortcut.exit_node_.dir_, path_down[0].n_, cost_t{120U});
      
      for(std::size_t i = 1; i < path_down.size(); ++i) {
        single_costs += w.r_->get_edge_cost<false>(path_down[i - 1].n_, path_down[i - 1].way_, path_down[i - 1].dir_, 
                                                     path_down[i].n_, cost_t{120U});
      }
      ASSERT_EQ(single_costs, expected_cost);
    }
  }
}

TEST(extract, pack_shortcuts) {
  auto const data_dir = "test/aachen.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {});
  auto w = ways{p, cista::mmap::protection::READ};
  for (auto const [rank, node] : utl::enumerate(w.r_->contraction_order_)) {
    for (auto [t_idx, target] : utl::enumerate(w.r_->sc_targets_[rank])) {

      auto const& cost_up = w.r_->cch_cost_up_[rank][t_idx];
      auto const& cost_down = w.r_->cch_cost_down_[rank][t_idx];
      auto const& sc_up = w.r_->cch_sc_up_[rank][t_idx];
      auto const& sc_down = w.r_->cch_sc_down_[rank][t_idx];

      utl::verify(w.r_->node_importance_[target] > rank, 
                  "[TARGET RANK VERIFY] Found importance {} of {} as target of node {} ({})",
                  w.r_->node_importance_[target], w.node_to_osm_[target], w.node_to_osm_[node], rank);
        
      utl::verify((!sc_up.is_valid() && cost_up == osr::kInfeasible) ||
                  (sc_up.is_valid() && cost_up != osr::kInfeasible),
                  "[SC COST VERIFY UP] Got cost {} and valid shortcut: {} from {} to {}", 
                  cost_up, sc_up.is_valid(), w.node_to_osm_[node], w.node_to_osm_[target]);

      utl::verify((!sc_down.is_valid() && cost_down == osr::kInfeasible) ||
                  (sc_down.is_valid() && cost_down != osr::kInfeasible),
                  "[SC COST VERIFY DOWN] Got cost {} and valid shortcut: {} from {} to {}",
                  cost_down, sc_down.is_valid(), w.node_to_osm_[node], w.node_to_osm_[target]);

      utl::verify((sc_up.entry_node_.n_ == node && sc_up.exit_node_.n_ == target) ||
                  (cost_up == osr::kInfeasible && sc_up.entry_node_.n_ == osr::node_idx_t::invalid() &&
                   sc_up.exit_node_.n_ == osr::node_idx_t::invalid()), 
                  "[SC POINT VERIFY UP] Expected entry {} but got {} and exit {} but got {}",
                  w.node_to_osm_[node], w.node_to_osm_[sc_up.entry_node_.n_], w.node_to_osm_[target],
                  w.node_to_osm_[sc_up.exit_node_.n_]);
        
      utl::verify((sc_down.entry_node_.n_ == target && sc_down.exit_node_.n_ == node) ||
                  (cost_down == osr::kInfeasible && sc_down.entry_node_.n_ == osr::node_idx_t::invalid() &&
                   sc_down.exit_node_.n_ == osr::node_idx_t::invalid()),
                  "[SC POINT VERIFY DOWN] Expected entry {} but got {} and exit {} but got {}",
                  w.node_to_osm_[target], w.node_to_osm_[sc_down.entry_node_.n_], w.node_to_osm_[node],
                  w.node_to_osm_[sc_down.exit_node_.n_]);
    } 
  }
}

TEST(extract, unpack_bigger_shortcut_forward) {
  auto const data_dir = "test/darmstadt-bismarckstr.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {});
  auto w = ways{p, cista::mmap::protection::READ};

  auto const& sc_1_up = w.r_->cch_sc_up_[83][0]; // 83 -> 134
  auto const path_1_up = w.r_->unpack_shortcut<direction::kForward, true>(sc_1_up);
  ASSERT_EQ(path_1_up.size(), 1);
  ASSERT_EQ(path_1_up[0], sc_1_up.exit_node_);

  auto const& sc_1_down = w.r_->cch_sc_down_[83][0];
  auto const path_1_down = w.r_->unpack_shortcut<direction::kForward, false>(sc_1_down);
  ASSERT_EQ(path_1_down.size(), 1);
  ASSERT_EQ(path_1_down[0], sc_1_down.exit_node_);

  auto const& sc_3_up = w.r_->cch_sc_up_[134][2];
  auto const path_3_up = w.r_->unpack_shortcut<direction::kForward, true>(sc_3_up);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(path_3_up[0], w.r_->cch_sc_down_[83][0].exit_node_);
  ASSERT_EQ(path_3_up[1], w.r_->cch_sc_up_[83][1].exit_node_);

  auto const& sc_3_down = w.r_->cch_sc_down_[134][2];
  auto const path_3_down = w.r_->unpack_shortcut<direction::kForward, false>(sc_3_down);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(path_3_down[0], w.r_->cch_sc_down_[83][1].exit_node_);
  ASSERT_EQ(path_3_down[1], w.r_->cch_sc_up_[83][0].exit_node_);

  auto const& sc_8_up = w.r_->cch_sc_up_[137][2];
  auto const path_8_up = w.r_->unpack_shortcut<direction::kForward, true>(sc_8_up);
  ASSERT_EQ(path_8_up.size(), 3);
  ASSERT_EQ(path_8_up[0], w.r_->cch_sc_down_[134][1].exit_node_);
  ASSERT_EQ(path_8_up[1], w.r_->cch_sc_down_[83][0].exit_node_);
  ASSERT_EQ(path_8_up[2], w.r_->cch_sc_up_[83][1].exit_node_);

  auto const& sc_8_down = w.r_->cch_sc_down_[137][2];
  auto const path_8_down = w.r_->unpack_shortcut<direction::kForward, false>(sc_8_down);
  ASSERT_EQ(path_8_down.size(), 3);
  ASSERT_EQ(path_8_down[0], w.r_->cch_sc_down_[83][1].exit_node_);
  ASSERT_EQ(path_8_down[1], w.r_->cch_sc_up_[83][0].exit_node_);
  ASSERT_EQ(path_8_down[2], w.r_->cch_sc_up_[134][1].exit_node_);

  auto const& sc_9_up = w.r_->cch_sc_up_[168][5];
  auto const path_9_up = w.r_->unpack_shortcut<direction::kForward, true>(sc_9_up);
  ASSERT_EQ(path_9_up.size(), 5);
  ASSERT_EQ(path_9_up[0], w.r_->cch_sc_down_[83][1].exit_node_);
  ASSERT_EQ(path_9_up[1], w.r_->cch_sc_up_[83][0].exit_node_);
  ASSERT_EQ(path_9_up[2], w.r_->cch_sc_up_[134][1].exit_node_);
  ASSERT_EQ(path_9_up[3], w.r_->cch_sc_down_[133][0].exit_node_);
  ASSERT_EQ(path_9_up[4], w.r_->cch_sc_up_[133][3].exit_node_);

  auto const& sc_9_down = w.r_->cch_sc_down_[168][5];
  auto const path_9_down = w.r_->unpack_shortcut<direction::kForward, false>(sc_9_down);
  ASSERT_EQ(path_9_down.size(), 5);
  ASSERT_EQ(path_9_down[0], w.r_->cch_sc_down_[133][3].exit_node_);
  ASSERT_EQ(path_9_down[1], w.r_->cch_sc_up_[133][0].exit_node_);
  ASSERT_EQ(path_9_down[2], w.r_->cch_sc_down_[134][1].exit_node_);
  ASSERT_EQ(path_9_down[3], w.r_->cch_sc_down_[83][0].exit_node_);
  ASSERT_EQ(path_9_down[4], w.r_->cch_sc_up_[83][1].exit_node_);
}

TEST(extract, unpack_bigger_shortcut_backward) {
  auto const data_dir = "test/darmstadt-bismarckstr.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {});
  auto w = ways{p, cista::mmap::protection::READ};

  auto const& sc_1_up = w.r_->cch_sc_up_[83][0];
  auto const path_1_up = w.r_->unpack_shortcut<direction::kBackward, true>(sc_1_up);
  ASSERT_EQ(path_1_up.size(), 1);
  ASSERT_EQ(path_1_up[0], w.r_->cch_sc_up_[83][0].entry_node_);

  auto const& sc_1_down = w.r_->cch_sc_down_[83][0];
  auto const path_1_down = w.r_->unpack_shortcut<direction::kBackward, false>(sc_1_down);
  ASSERT_EQ(path_1_down.size(), 1);
  ASSERT_EQ(path_1_down[0], sc_1_down.entry_node_);

  auto const& sc_3_up = w.r_->cch_sc_up_[134][2];
  auto const path_3_up = w.r_->unpack_shortcut<direction::kBackward, true>(sc_3_up);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(path_3_up[0], w.r_->cch_sc_down_[83][0].entry_node_);
  ASSERT_EQ(path_3_up[1], w.r_->cch_sc_up_[83][1].entry_node_);

  auto const& sc_3_down = w.r_->cch_sc_down_[134][2];
  auto const path_3_down = w.r_->unpack_shortcut<direction::kBackward, false>(sc_3_down);
  ASSERT_EQ(path_3_down.size(), 2);
  ASSERT_EQ(path_3_down[0], w.r_->cch_sc_down_[83][1].entry_node_);
  ASSERT_EQ(path_3_down[1], w.r_->cch_sc_up_[83][0].entry_node_);
}

TEST(extract, contraction_neighbor_init) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  //auto con = cch::contraction{w.r_};
  //con.build_contraction_order();
  //con.init_neighborhoods();
  
  // Test empty neighborhood
  ASSERT_EQ(w.r_->sc_targets_[19851].size(), 0);
  
  // neighborhood with one higher neighbor: Aachen - Dennewartstrasse
  ASSERT_EQ(w.r_->contraction_order_[1685], osr::node_idx_t{17169});
  ASSERT_EQ(w.r_->contraction_order_[1686], osr::node_idx_t{17170});
  ASSERT_EQ(w.r_->sc_targets_[1685].size(), 0);
  //ASSERT_EQ(w.r_->sc_targets_[1685][0], osr::node_idx_t{17170});

  // bigger neighborhood Aachen - Gabelung Büchel
  ASSERT_EQ(w.r_->contraction_order_[14241], osr::node_idx_t{14653});
  ASSERT_EQ(w.r_->contraction_order_[14245], osr::node_idx_t{2761});
  ASSERT_EQ(w.r_->contraction_order_[14251], osr::node_idx_t{201});
  ASSERT_EQ(w.r_->contraction_order_[14283], osr::node_idx_t{200});
  ASSERT_EQ(w.r_->sc_targets_[14241].size(), 3);
  ASSERT_EQ(w.r_->sc_targets_[14241][0], osr::node_idx_t{201});
  ASSERT_EQ(w.r_->sc_targets_[14241][1], osr::node_idx_t{2761});
  ASSERT_EQ(w.r_->sc_targets_[14241][2], osr::node_idx_t{200});
}

TEST(extract, contraction_filter_and_sort) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto con = cch::contraction{w.r_};
  con.build_contraction_order();
  con.init_neighborhoods();
  w.r_->sc_targets_[14241].push_back(osr::node_idx_t{201});
  w.r_->sc_targets_[14241].push_back(osr::node_idx_t{201});
  w.r_->sc_targets_[14241].push_back(osr::node_idx_t{200});
  con.sort_and_filter_neighbors(14241);

  auto const& probe = w.r_->sc_targets_[14241]; 
  ASSERT_EQ(probe[0], osr::node_idx_t{2761});
  ASSERT_EQ(probe[1], osr::node_idx_t{201});
  ASSERT_EQ(probe[2], osr::node_idx_t{200});
  ASSERT_EQ(probe.size(), 3);
}

TEST(shortcuts, initialization) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  auto valid_shortcut = cch::sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
  auto invalid_shortcut = cch::sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};

  ASSERT_EQ(valid_shortcut.get_path_cost(), osr::kInfeasible);

  valid_shortcut.add(osr::node_idx_t{0}, osr::way_idx_t{0}, osr::direction::kForward, osr::cost_t{3});
  invalid_shortcut.add(osr::node_idx_t{5}, osr::way_idx_t::invalid(), osr::direction::kBackward, osr::kInfeasible);

  ASSERT_EQ(valid_shortcut.nodes_.size(), 1);
  ASSERT_EQ(invalid_shortcut.nodes_.size(), 1);

  ASSERT_EQ(valid_shortcut.nodes_.back(), osr::node_idx_t{0});
  ASSERT_EQ(valid_shortcut.ways_.back(), osr::way_idx_t{0});
  ASSERT_EQ(valid_shortcut.dirs_.back(), osr::direction::kForward);
  ASSERT_EQ(valid_shortcut.costs_.back(), osr::cost_t{3});
  ASSERT_EQ(valid_shortcut.valid_, true);

  ASSERT_EQ(invalid_shortcut.nodes_.back(), osr::node_idx_t{5});
  ASSERT_EQ(invalid_shortcut.ways_.back(), osr::way_idx_t::invalid());
  ASSERT_EQ(invalid_shortcut.dirs_.back(), osr::direction::kBackward);
  ASSERT_EQ(invalid_shortcut.costs_.back(), osr::kInfeasible);
  ASSERT_EQ(invalid_shortcut.valid_, false);

  ASSERT_EQ(valid_shortcut.get_path_cost(), osr::cost_t{3});
  ASSERT_EQ(invalid_shortcut.get_path_cost(), osr::kInfeasible);
}

TEST(shortcuts, extension) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  auto valid_shortcut = cch::sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
  auto valid_appendice = cch::sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};

  valid_shortcut.add(osr::node_idx_t{0}, osr::way_idx_t{0}, osr::direction::kForward, osr::cost_t{3});
  valid_appendice.add(osr::node_idx_t{1}, osr::way_idx_t{2}, osr::direction::kBackward, osr::cost_t{4});

  auto valid_valid = valid_shortcut;
  valid_valid.append(valid_appendice, osr::cost_t{0U});

  ASSERT_EQ(valid_valid.nodes_.size(), 2);
  ASSERT_EQ(valid_valid.nodes_[0], osr::node_idx_t{0});
  ASSERT_EQ(valid_valid.nodes_[1], osr::node_idx_t{1});
  ASSERT_EQ(valid_valid.ways_[0], osr::way_idx_t{0});
  ASSERT_EQ(valid_valid.ways_[1], osr::way_idx_t{2});
  ASSERT_EQ(valid_valid.costs_[0], osr::cost_t{3});
  ASSERT_EQ(valid_valid.costs_[1], osr::cost_t{4});
  ASSERT_EQ(valid_valid.get_path_cost(), osr::cost_t{7});
}