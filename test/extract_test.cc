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

namespace fs = std::filesystem;
using namespace osr;

bool test_neighbors(osr::vec<cch::neighbor> const& neighbors,
                    osr::neighbor_idx_t const& probe,
                    osr::node_idx_t const& node,
                    std::uint32_t const& rank,
                    osr::way_idx_t const& way) {
  return neighbors[probe].neighbor_ == node &&
         neighbors[probe].rank_ == rank &&
         neighbors[probe].edge_ == way;
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
  ASSERT_EQ(mip.neighborhoods_[1685].neighbors_.size(), 1);
  ASSERT_TRUE(test_neighbors(mip.all_neighbors_,
                             mip.neighborhoods_[1685].neighbors_[0], 
                             osr::node_idx_t{17170}, 
                             static_cast<std::uint32_t>(1686), 
                             osr::way_idx_t{13036}));
  //ASSERT_EQ(mip.all_neighbors_[1685].neighbors_[0], std::tuple(osr::node_idx_t{17170}, static_cast<std::uint32_t>(1686), false, osr::node_idx_t{17170}, osr::way_idx_t{13036}, osr::way_idx_t{13036}));

  // Test neighborhood with multiple neighbors: Aachen - Gabelung Büchel
  ASSERT_EQ(mip.neighborhoods_[14241].node_, osr::node_idx_t{14653});
  ASSERT_EQ(mip.neighborhoods_[14245].node_, osr::node_idx_t{2761});
  ASSERT_EQ(mip.neighborhoods_[14251].node_, osr::node_idx_t{201});
  ASSERT_EQ(mip.neighborhoods_[14283].node_, osr::node_idx_t{200});
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

TEST(extract, customization) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/aachen.osm.pbf", p, {});
  auto w = ways{p, cista::mmap::protection::READ};
  auto l = lookup{w, p, cista::mmap::protection::READ};
  auto mip_proc = cch::mip_proc{w};
  mip_proc.build_contraction_order();
  mip_proc.init_neighborhoods();
  mip_proc.contract_nodes();
  auto profile = search_profile::kCar;
  auto params = get_parameters(profile);
  auto customization = cch::basic_customization{w, mip_proc};
  customization.run(profile, params);
  auto failed = std::uint64_t{0U};
  auto correct = std::uint64_t{0U};
  auto total = std::uint64_t{0U};

  auto const dijkstra_cost = [&](node_idx_t const from, 
                                 node_idx_t const to, 
                                 direction dir, 
                                 cost_t const val_up,
                                 std::uint64_t& failed,
                                 std::uint64_t& correct) {
    auto const from_loc = location{w.get_node_pos(from)};
    auto const to_loc = location{w.get_node_pos(to)};

    auto const node_pinned_matches = 
      [&](location const& loc, node_idx_t const n, bool const reverse) {
        auto matches = l.match<car>(car::parameters{}, loc, reverse, dir,
                                    100, nullptr);
        std::erase_if(matches, [&](auto const& wc){
          return wc.left_.node_ != n && wc.right_.node_ != n;
        });
        return matches;
      };
    
    auto const from_matches = node_pinned_matches(from_loc, from, false);
    auto const to_matches = node_pinned_matches(to_loc, to, true);
    auto const from_matches_span =
      std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = 
      std::span{begin(to_matches), end(to_matches)};
    
    auto const reference = [&]() {
      try {
        return route(car::parameters{}, w, l, search_profile::kCar, from_loc,
          to_loc, from_matches_span, to_matches_span, 2 * 3600U, dir, 
          nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
      } catch (std::exception const& ex) {
        fmt::println("dijkstra exception: {}", ex.what());
        throw ex;
      }
    }();

    if (reference.has_value()) {
      if (reference->cost_ == val_up) {
        ++correct;
      } else {
        ++failed;
      }
      ASSERT_EQ(reference->cost_, val_up);
    }
  };
  for (std::size_t nidx = 0; nidx <= 100; ++nidx) {
    auto const node = mip_proc.neighborhoods_[nidx];
    total += node.neighbors_.size();
    for (auto const neighbor : node.neighbors_) {
      auto const& neighbor_struct = mip_proc.all_neighbors_[neighbor];
      if (neighbor_struct.to_via_id_ == 0 &&
          neighbor_struct.to_neighbor_id_ == 0 &&
          neighbor_struct.via_ == osr::node_idx_t{0}) {
        auto const test_neighbor = mip_proc.all_neighbors_[neighbor];
        dijkstra_cost(node.node_, 
                test_neighbor.neighbor_, 
                //customization.dir_in_neighbor_[neighbor],
                direction::kForward,
                customization.neighbor_costs_up_[neighbor],
                failed,
                correct);
      }
    }
  }
  std::cout << "correct: " << correct << "\nfailed: " << failed << "\nof total: " << total;
}
// TEST(extract, shortcuts) {
//   auto p = fs::temp_directory_path() / "osr_test";
//   auto ec = std::error_code{};
//   fs::remove_all(p, ec);
//   fs::create_directories(p, ec);

//   extract(false, "test/aachen.osm.pbf", p, {});
//   auto w = ways{p, cista::mmap::protection::READ};

//   for (auto const s : w.r_->shortcut_properties_) {
//     ASSERT_TRUE(w.r_->node_importance_[s.via_] < w.r_->node_importance_[s.lower_end_]);
//     ASSERT_TRUE(w.r_->node_importance_[s.lower_end_] < w.r_->node_importance_[s.upper_end_]);
//   }
// }

// TEST(extract, neighborhood_concat) {
//   auto p = fs::temp_directory_path() / "osr_test";
//   auto ec = std::error_code{};
//   fs::remove_all(p, ec);
//   fs::create_directories(p, ec);

//   extract(false, "test/aachen.osm.pbf", p, {});

//   auto w = ways{p, cista::mmap::protection::READ};
//   auto mip = cch::mip_proc{w};

//   auto ex1_n = cch::neighborhood(osr::node_idx_t{10}, static_cast<std::uint32_t>(13));
//   auto ex2_n = cch::neighborhood(osr::node_idx_t{11}, static_cast<std::uint32_t>(12));

//   mip.concatenate_neighbors(ex1_n, ex2_n);
//   ASSERT_TRUE(ex2_n.neighbors_.empty());

//   ex2_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
//   ex2_n.neighbors_.push_back(std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(2)));

//   ex1_n.concatenate(ex2_n.neighbors_);
//   ASSERT_EQ(ex1_n.neighbors_[0], std::pair(osr::node_idx_t{2}, static_cast<std::uint32_t>(28)));
//   ASSERT_TRUE(ex1_n.neighbors_.size() == 1);
// }


// TEST(extract, elimination_tree) {
//   auto p = fs::temp_directory_path() / "osr_test";
//   auto ec = std::error_code{};
//   fs::remove_all(p, ec);
//   fs::create_directories(p, ec);

//   extract(false, "test/aachen.osm.pbf", p, {});

//   auto w = ways{p, cista::mmap::protection::READ};
//   auto mip = cch::mip_proc{w};
//   mip.build_contraction_order();
//   mip.init_neighborhoods();
//   mip.contract_nodes();

//   // neighbor without any neighbors:
//   ASSERT_EQ(mip.elimination_tree_[0], static_cast<std::uint32_t>(9663));
//   ASSERT_EQ(mip.elimination_tree_[1], static_cast<std::uint32_t>(2));
// }