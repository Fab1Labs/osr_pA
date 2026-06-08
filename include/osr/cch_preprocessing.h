#pragma once

#include <numeric>
#include <algorithm>

#include "cista/containers/vector.h"
#include "cista/strong.h"
#include "utl/enumerate.h"

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

struct g_plus_edge {
  osr::node_idx_t from_;
  osr::node_idx_t to_;
  bool is_shortcut_;
  osr::vec<osr::way_idx_t> replaces_;
};

struct g_plus {

  osr::vec_map<osr::edge_idx_t, g_plus_edge> edge_idx_to_data_; // edge information edge_idx_t => (id, start, end, is_shortcut, [wayIdx])
  // start Idx to id => efficient storage
};

struct mip_proc {
  explicit mip_proc(osr::ways const&);

  void test_contraction_order();
  void build_contraction_order();
  osr::vec<osr::node_idx_t> find_neighbors(osr::node_idx_t const&);
  std::uint32_t find_smallest_neighbor(osr::vec<osr::node_idx_t> const&);
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  bool is_in(osr::vec<osr::node_idx_t> const&, osr::node_idx_t const&);
  void perform_contraction();

  osr::ways const& ways_;
  osr::vec<osr::node_idx_t> contr_order_;
  osr::vec<std::uint32_t> elimination_tree_;
  g_plus g_plus_up_;

};

} //namespace cch