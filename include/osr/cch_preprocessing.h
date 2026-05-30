#pragma once

#include <numeric>

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

struct mip_proc {
  mip_proc(osr::ways const&);

  void test_contraction_order();
  void build_contraction_order();

  osr::ways const& ways_;
  osr::vec<osr::node_idx_t>contr_order_;

};

} //namespace cch