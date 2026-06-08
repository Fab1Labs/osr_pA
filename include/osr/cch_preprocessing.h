#pragma once

#include <numeric>
#include <algorithm>

#include "cista/containers/vector.h"
#include "cista/strong.h"
#include "utl/enumerate.h"

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

// struct g_plus {
//   explicit g_plus(osr::node_idx_t::value_t const&);

//   osr::node_idx_t::value_t const& size_;
//   osr::vecvec<std::uint32_t, osr::node_idx_t> neighbors_; // neighbors for node with rank
// }

struct mip_proc {
  explicit mip_proc(osr::ways const&);

  void test_contraction_order();
  void build_contraction_order();
  osr::vec<osr::node_idx_t> find_neighbors(osr::node_idx_t const&);
  std::uint32_t find_smallest_neighbor(osr::vec<osr::node_idx_t> const&);
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  void perform_contraction();

  osr::ways const& ways_;
  osr::vec<osr::node_idx_t> contr_order_;
  osr::vec<std::uint32_t> elimination_tree_;
  //cch::g_plus g_plus_up_;

};

} //namespace cch