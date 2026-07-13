#pragma once

#include "osr/types.h"


namespace cch {

struct shortcut_properties {

  osr::node_idx_t lower_end_;
  osr::node_idx_t via_;
  osr::node_idx_t upper_end_;
  osr::shortcut_idx_t lower_via_; // 
  osr::shortcut_idx_t via_upper_; // -> if zero, we have a direct way
};
} // namespace cch