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
  bool go_fwd_; // <- store if we can go this path forward
  bool go_bckwd_; // <- store if we can go this path back
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

} //namespace cch