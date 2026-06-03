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
  auto i_ = 0;
  for (auto const& node : ways_.r_->node_importance_) {
    contr_order_[node] = osr::node_idx_t{i_};
    ++i_;
  }
}

osr::vec<osr::node_idx_t> cch::mip_proc::find_neighbors(osr::node_idx_t const& node) {
  osr::vec<osr::node_idx_t> neighbors_;

  auto const& in_ways_ = ways_.r_->node_ways_[node];
  auto const& idx_in_ways_ = ways_.r_->node_in_way_idx_[node];
  if (in_ways_.empty() && idx_in_ways_.empty()) {return neighbors_;}

  //std::cout << "size " << in_ways_.size() << "\n";

  for (std::size_t i = 0; i < in_ways_.size(); ++i) {
    if (idx_in_ways_[i] > 0) {neighbors_.push_back(ways_.r_->way_nodes_[in_ways_[i]][idx_in_ways_[i] - 1]);}
    if (idx_in_ways_[i] < ways_.r_->way_nodes_.size() - 1) {neighbors_.push_back(ways_.r_->way_nodes_[in_ways_[i]][idx_in_ways_[i] + 1]);}

    //std::cout << in_ways_[i] << " " << idx_in_ways_[i] << "\n";
  }

  for (auto const& n : neighbors_){std::cout << n << " ";}
  return neighbors_;
}