#pragma once

#include <numeric>
#include <algorithm>
#include <utility>

#include "cista/containers/vector.h"
#include "cista/strong.h"
#include "cista/io.h"
#include "utl/enumerate.h"

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

struct neighbor {
  bool operator==(const cch::neighbor&) const = default;

  osr::node_idx_t neighbor_;
  std::uint32_t rank_;
  osr::node_idx_t via_;
  std::uint64_t to_via_id_;
  std::uint64_t to_neighbor_id_;
  osr::way_idx_t edge_;
  std::uint16_t in_way_idx_;
  osr::direction dir_;
  bool go_up_; // <- store if we can go this path upward
  bool go_dwn_; // <- store if we can go this path downward
};

struct neighborhood {

  neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank);
  void sort_neighbors(osr::vec<neighbor>&);
  //bool is_neighbor(osr::node_idx_t const&);

  osr::node_idx_t const node_;
  std::uint32_t const rank_;
  osr::vec<osr::neighbor_idx_t> neighbors_;
  osr::vec<osr::shortcut_idx_t> shortcuts_;
};

struct mip_proc {
  explicit mip_proc(osr::ways&);
 
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  bool is_in(osr::vec<osr::neighbor_idx_t>& neighbors, osr::node_idx_t const& node);
  bool is_shortcut(neighbor const&);
  bool is_neighbor(neighborhood const& nhood, osr::node_idx_t const& n);
  bool accessible_way(osr::way_idx_t const& w, osr::direction const& d);
  bool accessible_node(osr::node_idx_t const& n);

  void build_contraction_order();
  void init_neighborhoods();
  void concatenate_neighbors(neighborhood const& pred, neighborhood& succ);
  void contract_nodes();
  void filter_neighborhoods();
  void write_shortcuts(cista::mmap::protection);

  cista::mmap mm(char const* file, cista::mmap::protection mode) {
    return cista::mmap{(ways_.p_ / file).generic_string().c_str(), mode};
  }

  osr::ways& ways_;
  osr::vec<osr::node_idx_t> contr_order_;
  osr::vec<neighborhood> neighborhoods_; // <- sorted by rank
  osr::vec<neighbor> all_neighbors_; // <- identifiable by idx
  osr::vec<bool> is_accessible_up_;
  osr::vec<bool> is_accessible_down_;
  osr::vec<std::uint16_t> in_way_idx_;
  osr::vec<std::uint16_t> out_way_idx_;
  osr::vec<std::uint32_t> elimination_tree_;
  std::uint64_t max_neighbors_;
};

struct contraction {
  explicit contraction(cista::wrapped<osr::ways::routing>&);

  // helper functions:
  void build_contraction_order() {
    r_->contraction_order_.resize(r_->node_importance_.size());
    for (auto const [i, rank] : utl::enumerate(r_->node_importance_)) {
      r_->contraction_order_[rank] = osr::node_idx_t{i};
    }
  }

  bool accessible_way(osr::way_idx_t const& w, osr::direction const& d) {
    auto const& wp = r_->way_properties_[w];
    return wp.is_car_accessible() && (d == osr::direction::kForward || !wp.is_oneway_car());
  }

  bool accessible_node(osr::node_idx_t const& n) {
    auto const& np = r_->node_properties_[n];
    return np.is_car_accessible();
  }

  // sort the neighbors of a node by rank
  void sort_and_filter_neighbors(std::uint32_t const& rank) {
    if (r_->sc_targets_[rank].size() < 2) { return; }
    auto sorting_condition = [this](osr::node_idx_t const lhs, osr::node_idx_t const rhs) {
      return r_->node_importance_[lhs] < r_->node_importance_[rhs];
    };
    // sort the given neighbors
    std::sort(r_->sc_targets_[rank].begin(), r_->sc_targets_[rank].end(), sorting_condition);
    // filter duplicates
    auto last_s = std::unique(r_->sc_targets_[rank].begin(), r_->sc_targets_[rank].end());
    r_->sc_targets_[rank].erase(last_s, r_->sc_targets_[rank].end());

    for (std::size_t i = 0; i < (r_->sc_targets_[rank].size() - 1); ++i) {
      utl::verify(r_->node_importance_[r_->sc_targets_[rank][i]] < r_->node_importance_[r_->sc_targets_[rank][i + 1]],
                  "Neighbors are sorted incorrectly");
    }
    utl::verify(rank < r_->node_importance_[r_->sc_targets_[rank][0]], 
                "Got false lowest higher neighbor of rank {} at next with rank {}", 
                r_->node_importance_[r_->sc_targets_[rank][0]], rank);
  }

  // main functions:
  void init_neighborhoods();
  void contract_nodes();

  cista::wrapped<osr::ways::routing>& r_;
};

} //namespace cch