#pragma once

#include "osr/types.h"


namespace cch {

struct shortcut_properties {

  osr::node_idx_t lower_end_;
  osr::node_idx_t via_;
  osr::node_idx_t upper_end_;
  osr::way_idx_t lower_via_;
  osr::way_idx_t via_upper_;
};
} // namespace cch