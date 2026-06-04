#include "osr/cch_preprocessing.h"

#include <iostream>

#include "osr/ways.h"


cch::mip_proc::mip_proc(osr::ways const& w)// std::filesystem::path p, cista::mmap::protection const mode)
  : ways_{w} {}

void cch::mip_proc::test_contraction_order() {
  auto importance_copy_ = ways_.r_->node_importance_;
  importance_copy_.resize(20);
  for (auto const& e : importance_copy_) {std::cout << e << " " << contr_order_[e] << "\n";}
}

// define the contraction order for the preprocessing here
void cch::mip_proc::build_contraction_order() {
  contr_order_.resize(ways_.n_nodes());
  for (auto const [i, node] : utl::enumerate(ways_.r_->node_importance_)) {
    contr_order_[node] = osr::node_idx_t{i};
  }
}

osr::vec<osr::node_idx_t> cch::mip_proc::find_neighbors(osr::node_idx_t const& node) {
  osr::vec<osr::node_idx_t> neighbors_;

  // check for existing neighbors
  auto const& in_ways_ = ways_.r_->node_ways_[node];
  auto const& idx_in_ways_ = ways_.r_->node_in_way_idx_[node];
  if (in_ways_.empty() && idx_in_ways_.empty()) {return neighbors_;}

  // add existing neighbors with higher rank:
  for (auto const [idx, way] : utl::zip(idx_in_ways_, in_ways_)) {
    if (idx > 0) {
      auto const& pred = ways_.r_->way_nodes_[way][idx - 1];
      if (check_importance(node, pred)) {neighbors_.push_back(pred);}
    }
    if (idx < ways_.r_->way_nodes_.size() - 1) {
      auto const& succ = ways_.r_->way_nodes_[way][idx + 1];
      if (check_importance(node, succ)) {neighbors_.push_back(succ);}
    }
  }

  for (auto const& n : neighbors_){std::cout << n << " ";}
  return neighbors_;
}

bool cch::mip_proc::check_importance(osr::node_idx_t const& lhs, osr::node_idx_t const& rhs) {
  //std::cout << "Importance lhs: " << ways_.r_->node_importance_[lhs] << " Importance rhs: " << ways_.r_->node_importance_[rhs] << "\n";
  return ways_.r_->node_importance_[lhs] < ways_.r_->node_importance_[rhs];
}