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
  osr::distance_t dist_;
  osr::direction dir_;
  std::uint64_t id_;
};

struct neighborhood {

  neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank);
  void sort_neighbors();
  void filter_higher_neighbors();
  bool is_neighbor(osr::node_idx_t const&);

  osr::node_idx_t const node_;
  std::uint32_t const rank_;
  osr::vec<osr::neighbor_idx_t> neighbors_;
};

struct mip_proc {
  explicit mip_proc(osr::ways&);
 
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  bool is_in(osr::vec<neighbor> const& neighbors, osr::node_idx_t const& node);
  bool is_shortcut(neighbor const&);

  void build_contraction_order();
  void init_neighborhoods();
  void concatenate_neighbors(neighborhood const& pred, neighborhood& succ);
  void contract_nodes();
  void write_shortcuts(cista::mmap::protection);;

  cista::mmap mm(char const* file, cista::mmap::protection mode) {
    return cista::mmap{(ways_.p_ / file).generic_string().c_str(), mode};
  }

  osr::ways& ways_;
  osr::vec<osr::node_idx_t> contr_order_;
  osr::vec<neighborhood> neighborhoods_; // <- sorted by rank
  osr::vec<neighbor> all_neighbors_; // <- identifiable by id_
  osr::vec<std::uint32_t> elimination_tree_;
  std::uint64_t neighbor_counter_;
};

} //namespace cch