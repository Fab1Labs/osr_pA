#pragma once

#include "osr/types.h"


namespace cch {

struct shortcut_properties {
  osr::node_idx_t lower_end_;
  osr::node_idx_t upper_end_;
  osr::node_idx_t via_;
  osr::shortcut_idx_t lower_via_; // 
  osr::shortcut_idx_t via_upper_; // -> if zero, we have a direct way
  osr::way_idx_t edge_;
  osr::direction dir_;
  std::uint16_t in_way_idx_;
};

struct shortcut_nav_infos {

  void change_direction() {
    dir_ = osr::opposite(dir_);
  }

  osr::way_idx_t way_;
  osr::direction dir_;
};

} // namespace cch