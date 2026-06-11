#pragma once

#include <numeric>
#include <algorithm>
#include <utility>

#include "cista/containers/vector.h"
#include "cista/strong.h"
#include "utl/enumerate.h"

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

struct neighborhood {

  neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank);
  void add_neighbor(osr::node_idx_t const& node, std::uint32_t const& rank);
  void sort_neighbors();

  osr::node_idx_t const& node_;
  std::uint32_t const& rank_;
  osr::vec<std::pair<osr::node_idx_t, std::uint32_t>> neighbors_;
};

struct mip_proc {
  explicit mip_proc(osr::ways const&);

  void test_contraction_order();
  void build_contraction_order();
  osr::vec<osr::node_idx_t> find_neighbors(osr::node_idx_t const&);
  //std::uint32_t find_smallest_neighbor(osr::vec<osr::node_idx_t> const&);
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  bool is_in(osr::vec<osr::node_idx_t> const&, osr::node_idx_t const&);
  void perform_contraction();

  osr::ways const& ways_;
  osr::vec<osr::node_idx_t> contr_order_;
  osr::vec<neighborhood> all_neighbors_;
  osr::vec<std::uint32_t> elimination_tree_;
};

} //namespace cch