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
  static constexpr sc_properties invalid(osr::node_idx_t const& n) noexcept {
    auto sc = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
    sc.add(n, 
           osr::way_idx_t::invalid(), 
           osr::direction::kForward, 
           osr::kInfeasible);
    return sc;  
  }

  void add(osr::node_idx_t const n,
              osr::way_idx_t const w,
              osr::direction const d,
              osr::cost_t const c) {
    nodes_.push_back(n);
    ways_.push_back(w);
    dirs_.push_back(d);
    costs_.push_back(c);
    valid_ = c != osr::kInfeasible;
  }

  osr::cost_t get_path_cost() const {
    if (costs_.size() == 0) {
      return osr::kInfeasible;
    }

    auto sum = osr::cost_t{0};
    for (auto c : costs_) {
      if (c != osr::kInfeasible) {
        sum += c;
      } else {
        sum = osr::kInfeasible;
        break;
      }
    }

    return sum;
  }

  void reverse_path(osr::node_idx_t const& t) {
    nodes_.pop_back(); // remove the downward target (current node for upward path)
    std::reverse(nodes_.begin(), nodes_.end());
    nodes_.push_back(t);
  }

  void append(sc_properties& other) {
    utl::verify(valid_ && other.valid_, "[SC APPEND] Tried to concatenate invalid shortcuts.");

    nodes_.insert(nodes_.end(), other.nodes_.begin(), other.nodes_.end());
    ways_.insert(ways_.end(), other.ways_.begin(), other.ways_.end());
    dirs_.insert(dirs_.end(), other.dirs_.begin(), other.dirs_.end());
    costs_.insert(costs_.end(), other.costs_.begin(),other.costs_.end());
    auto max_it = std::max_element(costs_.begin(), costs_.end());
    valid_ = (max_it != costs_.end()) && (*max_it != osr::kInfeasible);
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
  bool valid_;

  // car::node(node_idx_t, way_pos, dir), cost, way_idx_t,
};

} // namespace cch