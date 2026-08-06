#pragma once

#include <numeric>

#include "utl/verify.h"

#include "osr/types.h"
#include "osr/ways.h"

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


struct sc_properties{

  void add(osr::node_idx_t const n,
              osr::way_idx_t const w,
              osr::direction const d,
              osr::cost_t const c) {
    nodes_.push_back(n);
    ways_.push_back(w);
    dirs_.push_back(d);
    costs_.push_back(c);
  }

  osr::cost_t get_cost() {
    if (costs_.size() > 0) {
      return costs_.back();
    } else {
      return osr::kInfeasible;
    }
  }

  void reverse_path(osr::node_idx_t const& t) {
    nodes_.pop_back(); // remove the downward target (current node for upward path)
    std::reverse(nodes_.begin(), nodes_.end());
    nodes_.push_back(t);
  }

  void transform_costs() {
    // reverse costs
    std::reverse(costs_.begin(), costs_.end());
    // subtract predecessor costs
    for (std::size_t i = 0; i < (costs_.size() - 1); ++i) {
      costs_[i] -= costs_[i + 1];
    }
    // add up again
    for (std::size_t i = 1; i < costs_.size(); ++i) {
      costs_[i] += costs_[i - 1];
    }
  }

  void append(sc_properties& other) {
    nodes_.insert(nodes_.end(), other.nodes_.begin(), other.nodes_.end());
    ways_.insert(ways_.end(), other.ways_.begin(), other.ways_.end());
    dirs_.insert(dirs_.end(), other.dirs_.begin(), other.dirs_.end());
    
    auto new_costs = other.costs_;
    std::for_each(new_costs.begin(), new_costs.end(), [this](osr::cost_t& c){ c += costs_.back();});
    costs_.insert(costs_.end(), new_costs.begin(),new_costs.end());
    utl::verify(nodes_.size() == ways_.size() &&
                nodes_.size() == dirs_.size() && 
                nodes_.size() == costs_.size(), 
                "Path-Append Flaw: got n.size: {}, w.size(): {}, d.size(): {}, c.size(): {}",
                nodes_.size(), ways_.size(), dirs_.size(), costs_.size());
  };

  osr::vec<osr::node_idx_t> nodes_;
  osr::vec<osr::way_idx_t> ways_;
  osr::vec<osr::direction> dirs_;
  osr::vec<osr::cost_t> costs_;

  // car::node(node_idx_t, way_pos, dir), cost, way_idx_t,
};

} // namespace cch