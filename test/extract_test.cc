#include "gtest/gtest.h"

#include <filesystem>
#include <iostream>

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

TEST(extract, unpack_bigger_shortcut) {
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
  auto const path_1_up = w.r_->unpack_shortcut<true>(sc_1_up);
  ASSERT_EQ(path_1_up.size(), 1);
  ASSERT_EQ(path_1_up[0], sc_1_up.exit_node_);

  auto const& sc_1_down = w.r_->cch_sc_down_[83][0];
  auto const path_1_down = w.r_->unpack_shortcut<false>(sc_1_down);
  ASSERT_EQ(path_1_down.size(), 1);
  ASSERT_EQ(path_1_down[0], sc_1_down.exit_node_);

  auto const& sc_3_up = w.r_->cch_sc_up_[134][2];
  auto const path_3_up = w.r_->unpack_shortcut<true>(sc_3_up);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(path_3_up[0], w.r_->cch_sc_down_[83][0].exit_node_);
  ASSERT_EQ(path_3_up[1], w.r_->cch_sc_up_[83][1].exit_node_);

  auto const& sc_3_down = w.r_->cch_sc_down_[134][2];
  auto const path_3_down = w.r_->unpack_shortcut<false>(sc_3_down);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(path_3_down[0], w.r_->cch_sc_down_[83][1].exit_node_);
  ASSERT_EQ(path_3_down[1], w.r_->cch_sc_up_[83][0].exit_node_);

  auto const& sc_8_up = w.r_->cch_sc_up_[137][2];
  auto const path_8_up = w.r_->unpack_shortcut<true>(sc_8_up);
  ASSERT_EQ(path_8_up.size(), 3);
  ASSERT_EQ(path_8_up[0], w.r_->cch_sc_down_[134][1].exit_node_);
  ASSERT_EQ(path_8_up[1], w.r_->cch_sc_down_[83][0].exit_node_);
  ASSERT_EQ(path_8_up[2], w.r_->cch_sc_up_[83][1].exit_node_);

  auto const& sc_8_down = w.r_->cch_sc_down_[137][2];
  auto const path_8_down = w.r_->unpack_shortcut<false>(sc_8_down);
  ASSERT_EQ(path_8_down.size(), 3);
  ASSERT_EQ(path_8_down[0], w.r_->cch_sc_down_[83][1].exit_node_);
  ASSERT_EQ(path_8_down[1], w.r_->cch_sc_up_[83][0].exit_node_);
  ASSERT_EQ(path_8_down[2], w.r_->cch_sc_up_[134][1].exit_node_);

  auto const& sc_9_up = w.r_->cch_sc_up_[168][5];
  auto const path_9_up = w.r_->unpack_shortcut<true>(sc_9_up);
  ASSERT_EQ(path_9_up.size(), 5);
  ASSERT_EQ(path_9_up[0], w.r_->cch_sc_down_[83][1].exit_node_);
  ASSERT_EQ(path_9_up[1], w.r_->cch_sc_up_[83][0].exit_node_);
  ASSERT_EQ(path_9_up[2], w.r_->cch_sc_up_[134][1].exit_node_);
  ASSERT_EQ(path_9_up[3], w.r_->cch_sc_down_[133][0].exit_node_);
  ASSERT_EQ(path_9_up[4], w.r_->cch_sc_up_[133][3].exit_node_);

  auto const& sc_9_down = w.r_->cch_sc_down_[168][5];
  auto const path_9_down = w.r_->unpack_shortcut<false>(sc_9_down);
  ASSERT_EQ(path_9_down.size(), 5);
  ASSERT_EQ(path_9_down[0], w.r_->cch_sc_down_[133][3].exit_node_);
  ASSERT_EQ(path_9_down[1], w.r_->cch_sc_up_[133][0].exit_node_);
  ASSERT_EQ(path_9_down[2], w.r_->cch_sc_down_[134][1].exit_node_);
  ASSERT_EQ(path_9_down[3], w.r_->cch_sc_down_[83][0].exit_node_);
  ASSERT_EQ(path_9_down[4], w.r_->cch_sc_up_[83][1].exit_node_);
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
  ASSERT_EQ(mip.neighborhoods_[19851].node_, osr::node_idx_t{7884});
  ASSERT_TRUE(mip.neighborhoods_[19851].neighbors_.empty());

  // Test neighborhood with one higher neighbor: Aachen - Dennewartstrasse
  ASSERT_EQ(mip.neighborhoods_[1685].node_, osr::node_idx_t{17169});
  ASSERT_EQ(mip.neighborhoods_[1685].neighbors_.size(), 0); // <- falls mit accessible check getestet wird
  // ASSERT_EQ(mip.neighborhoods_[1685].neighbors_.size(), 1); // <- falls ohne accessible check getestet wird
  // ASSERT_TRUE(test_neighbors(mip.all_neighbors_,
  //                            mip.neighborhoods_[1685].neighbors_[0], 
  //                            osr::node_idx_t{17170}, 
  //                            static_cast<std::uint32_t>(1686), 
  //                            osr::way_idx_t{13036}));
  //ASSERT_EQ(mip.all_neighbors_[1685].neighbors_[0], std::tuple(osr::node_idx_t{17170}, static_cast<std::uint32_t>(1686), false, osr::node_idx_t{17170}, osr::way_idx_t{13036}, osr::way_idx_t{13036}));

  // Test neighborhood with multiple neighbors: Aachen - Gabelung Büchel
  ASSERT_EQ(mip.neighborhoods_[14241].node_, osr::node_idx_t{14653});
  ASSERT_EQ(mip.neighborhoods_[14245].node_, osr::node_idx_t{2761});
  ASSERT_EQ(mip.neighborhoods_[14251].node_, osr::node_idx_t{201});
  ASSERT_EQ(mip.neighborhoods_[14283].node_, osr::node_idx_t{200});
  ASSERT_EQ(mip.neighborhoods_[14241].neighbors_.size(), 3);
  
  ASSERT_TRUE(test_neighbors(mip.all_neighbors_,
                             mip.neighborhoods_[14241].neighbors_[0], 
                             osr::node_idx_t{201}, 
                             static_cast<std::uint32_t>(14251), 
                             osr::way_idx_t{6255}));
  ASSERT_TRUE(test_neighbors(mip.all_neighbors_,
                             mip.neighborhoods_[14241].neighbors_[1], 
                             osr::node_idx_t{2761}, 
                             static_cast<std::uint32_t>(14245), 
                             osr::way_idx_t{6255}));
  ASSERT_TRUE(test_neighbors(mip.all_neighbors_,
                             mip.neighborhoods_[14241].neighbors_[2], 
                             osr::node_idx_t{200}, 
                             static_cast<std::uint32_t>(14283), 
                             osr::way_idx_t{10495}));
  // ASSERT_EQ(mip.all_neighbors_[14241].neighbors_[0], std::tuple(osr::node_idx_t{201}, static_cast<std::uint32_t>(14251), false, osr::node_idx_t{201}, osr::way_idx_t{6255}, osr::way_idx_t{6255}));
  // ASSERT_EQ(mip.all_neighbors_[14241].neighbors_[1], std::tuple(osr::node_idx_t{2761}, static_cast<std::uint32_t>(14245), false, osr::node_idx_t{2761}, osr::way_idx_t{6255}, osr::way_idx_t{6255}));
  // ASSERT_EQ(mip.all_neighbors_[14241].neighbors_[2], std::tuple(osr::node_idx_t{200}, static_cast<std::uint32_t>(14283), false, osr::node_idx_t{200}, osr::way_idx_t{10495}, osr::way_idx_t{10495}));

}

TEST(extract, sort_neighbors) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  //extract(false, "test/aachen.osm.pbf", p, {});

  osr::vec<cch::neighbor> all_neighbors;
  auto ex1_n = cch::neighborhood{osr::node_idx_t{3}, static_cast<std::uint32_t>(23)};

  all_neighbors.push_back(cch::neighbor{.neighbor_ = osr::node_idx_t{1}, 
                                           .rank_ = static_cast<std::uint32_t>(36), 
                                           .via_ = osr::node_idx_t{3}, 
                                           .to_via_id_ = 0, 
                                           .to_neighbor_id_ = 0, 
                                           .edge_ = osr::way_idx_t{1},
                                           .in_way_idx_ = 0,
                                           .dir_ = osr::direction::kForward});
  all_neighbors.push_back(cch::neighbor{.neighbor_ = osr::node_idx_t{2}, 
                                           .rank_ = static_cast<std::uint32_t>(28), 
                                           .via_ = osr::node_idx_t{3}, 
                                           .to_via_id_ = 0,
                                           .to_neighbor_id_ = 0,
                                           .edge_ = osr::way_idx_t{2},
                                           .in_way_idx_ = 0,
                                           .dir_ = osr::direction::kForward});
  all_neighbors.push_back(cch::neighbor{.neighbor_ = osr::node_idx_t{4}, 
                                           .rank_ = static_cast<std::uint32_t>(27), 
                                           .via_ = osr::node_idx_t{3}, 
                                           .to_via_id_ = 0,
                                           .to_neighbor_id_ = 0,
                                           .edge_ = osr::way_idx_t{4},
                                           .in_way_idx_ = 0,
                                           .dir_ = osr::direction::kForward});
  
  ex1_n.neighbors_.push_back(osr::neighbor_idx_t{0});
  ex1_n.neighbors_.push_back(osr::neighbor_idx_t{1});
  ex1_n.neighbors_.push_back(osr::neighbor_idx_t{2});
  ex1_n.neighbors_.push_back(osr::neighbor_idx_t{1});
  //ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{1}, static_cast<std::uint32_t>(36), false, osr::node_idx_t{3}, osr::way_idx_t{1}, osr::way_idx_t{1}));
  // ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{2}, static_cast<std::uint32_t>(28), false, osr::node_idx_t{3}, osr::way_idx_t{2}, osr::way_idx_t{2}));
  // ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{4}, static_cast<std::uint32_t>(27), false, osr::node_idx_t{3}, osr::way_idx_t{4}, osr::way_idx_t{4}));
  // ex1_n.neighbors_.push_back(std::tuple(osr::node_idx_t{2}, static_cast<std::uint32_t>(28), false, osr::node_idx_t{3}, osr::way_idx_t{2}, osr::way_idx_t{2}));

  osr::vec<osr::neighbor_idx_t> ex1_n_exp;
  ex1_n_exp.push_back(osr::neighbor_idx_t{2});
  ex1_n_exp.push_back(osr::neighbor_idx_t{1});
  ex1_n_exp.push_back(osr::neighbor_idx_t{0});

  // ex1_n_exp.push_back(std::tuple(osr::node_idx_t{4}, static_cast<std::uint32_t>(27), false, osr::node_idx_t{3}, osr::way_idx_t{4}, osr::way_idx_t{4}));
  // ex1_n_exp.push_back(std::tuple(osr::node_idx_t{2}, static_cast<std::uint32_t>(28), false, osr::node_idx_t{3}, osr::way_idx_t{2}, osr::way_idx_t{2}));
  // ex1_n_exp.push_back(std::tuple(osr::node_idx_t{1}, static_cast<std::uint32_t>(36), false, osr::node_idx_t{3}, osr::way_idx_t{1}, osr::way_idx_t{1}));

  ex1_n.sort_neighbors(all_neighbors);
  ASSERT_EQ(ex1_n.neighbors_, ex1_n_exp);
}

TEST(extract, neighborhood_concat) {
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

  for (auto const n : mip.neighborhoods_) {
    for (auto const neighbor : n.neighbors_){
      ASSERT_TRUE(n.rank_ < mip.all_neighbors_[neighbor].rank_);
    }
  }
}

TEST(extract, contraction_order_new) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto con = cch::contraction{w.r_};
  con.build_contraction_order();

  bool eq = true;
  for (auto const [rank, node] : utl::enumerate(w.r_->contraction_order_)) {
    eq = eq && (static_cast<std::uint32_t>(rank) == w.r_->node_importance_[node]);
  }
  
  ASSERT_TRUE(eq);
  ASSERT_EQ(w.r_->contraction_order_.size(), 20063);
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
// TEST(extract, sc_properties_handling) {
//   auto p = fs::temp_directory_path() / "osr_test";
//   auto ec = std::error_code{};
//   fs::remove_all(p, ec);
//   fs::create_directories(p, ec);

//   auto simple_case = cch::sc_properties{
//     .nodes_ = {},
//     .ways_ = {},
//     .dirs_ = {},
//     .costs_ = {}
//   };

//   simple_case.add(osr::node_idx_t{0}, osr::way_idx_t{0}, osr::direction::kForward, osr::cost_t{3});

//   // Test correct initialization:
//   ASSERT_EQ(simple_case.nodes_[0], osr::node_idx_t{0});
//   ASSERT_EQ(simple_case.ways_[0], osr::way_idx_t{0});
//   ASSERT_EQ(simple_case.dirs_[0], osr::direction::kForward);
//   ASSERT_EQ(simple_case.costs_[0], osr::cost_t{3});

//   // Test cost function:
//   ASSERT_EQ(simple_case.get_cost(), osr::cost_t{3});

//   // add a new shortcutpath to extend:
//   auto appendice = cch::sc_properties{
//     .nodes_ = {},
//     .ways_ = {},
//     .dirs_ = {},
//     .costs_ = {}
//   };
//   appendice.add(osr::node_idx_t{1}, osr::way_idx_t{1}, osr::direction::kForward, osr::cost_t{4});

//   auto combined = simple_case;
//   combined.append(appendice);
//   ASSERT_EQ(simple_case.nodes_.size(), 1);
//   ASSERT_EQ(simple_case.ways_.size(), 1);
//   ASSERT_EQ(simple_case.dirs_.size(), 1);
//   ASSERT_EQ(simple_case.costs_.size(), 1);

//   ASSERT_EQ(combined.nodes_.size(), 2);
//   ASSERT_EQ(combined.nodes_[0], osr::node_idx_t{0});
//   ASSERT_EQ(combined.nodes_[1], osr::node_idx_t{1});
//   ASSERT_EQ(combined.ways_.size(), 2);
//   ASSERT_EQ(combined.dirs_.size(), 2);
//   ASSERT_EQ(combined.costs_.size(), 2);
//   ASSERT_EQ(combined.costs_[0], osr::cost_t{3});
//   ASSERT_EQ(combined.costs_[1], osr::cost_t{7});
//   auto reverse_example = cch::sc_properties{
//     .nodes_ = {},
//     .ways_ = {},
//     .dirs_ = {},
//     .costs_ = {}
//   };
//   reverse_example.add(osr::node_idx_t{1}, osr::way_idx_t{1}, osr::direction::kForward, osr::cost_t{1});
//   reverse_example.nodes_.push_back(osr::node_idx_t{2});
//   reverse_example.nodes_.push_back(osr::node_idx_t{3});
//   reverse_example.nodes_.push_back(osr::node_idx_t{4});
//   reverse_example.nodes_.push_back(osr::node_idx_t{5});
//   reverse_example.reverse_path(osr::node_idx_t{0});
//   ASSERT_EQ(reverse_example.nodes_[0], osr::node_idx_t{4});
//   ASSERT_EQ(reverse_example.nodes_[1], osr::node_idx_t{3});
//   ASSERT_EQ(reverse_example.nodes_[2], osr::node_idx_t{2});
//   ASSERT_EQ(reverse_example.nodes_[3], osr::node_idx_t{1});
//   ASSERT_EQ(reverse_example.nodes_[4], osr::node_idx_t{0});

//   reverse_example.costs_.push_back(osr::cost_t{3});
//   reverse_example.costs_.push_back(osr::cost_t{5});
//   reverse_example.costs_.push_back(osr::cost_t{7});
//   reverse_example.transform_costs();
//   ASSERT_EQ(reverse_example.costs_[0], osr::cost_t{2});
//   ASSERT_EQ(reverse_example.costs_[1], osr::cost_t{4});
//   ASSERT_EQ(reverse_example.costs_[2], osr::cost_t{6});
//   ASSERT_EQ(reverse_example.costs_[3], osr::cost_t{7});

//   simple_case.reverse_path(osr::node_idx_t{8});
//   ASSERT_EQ(simple_case.nodes_.size(), 1);
//   ASSERT_EQ(simple_case.nodes_[0], osr::node_idx_t{8});
//   simple_case.transform_costs();
//   ASSERT_EQ(simple_case.costs_.size(), 1);
//   ASSERT_EQ(simple_case.costs_[0], osr::cost_t{3});

//   auto invalid_case = cch::sc_properties{
//     .nodes_ = {},
//     .ways_ = {},
//     .dirs_ = {},
//     .costs_ = {}
//   };
//   invalid_case.add(osr::node_idx_t{0}, osr::way_idx_t::invalid(), 
//       osr::direction::kForward, osr::kInfeasible);
//   invalid_case.reverse_path(osr::node_idx_t{1});
//   invalid_case.transform_costs();
//   ASSERT_EQ(invalid_case.nodes_.size(), 1);
//   ASSERT_EQ(invalid_case.nodes_.back(), osr::node_idx_t{1});
//   ASSERT_EQ(invalid_case.costs_.size(), 1);
//   ASSERT_EQ(invalid_case.costs_.back(), osr::kInfeasible);
//   ASSERT_EQ(invalid_case.ways_[0], osr::way_idx_t::invalid());
// }

TEST(extract, find_way) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/darmstadt-bismarckstr.osm.pbf", p, {});
  auto w = ways{p, cista::mmap::protection::READ};
  auto customization = cch::customization(w.r_);

  auto const way_data = customization.find_way(osr::node_idx_t{222}, osr::node_idx_t{188});
  std::cout << "\nResults: " << way_data.way_ << " " << way_data.dir_ << " " << way_data.node_in_way_idx_;
  ASSERT_EQ(way_data.way_, osr::way_idx_t{135});
}