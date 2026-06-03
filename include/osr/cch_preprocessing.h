#pragma once

#include <numeric>

#include "cista/containers/vector.h"

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

struct mip_proc {
  explicit mip_proc(osr::ways const&);

  void test_contraction_order();
  void build_contraction_order();
  osr::vec<osr::node_idx_t> find_neighbors(osr::node_idx_t const&);
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  void add_shortcuts();

  osr::ways const& ways_;
  osr::vec<osr::node_idx_t> contr_order_;

};

} //namespace cch