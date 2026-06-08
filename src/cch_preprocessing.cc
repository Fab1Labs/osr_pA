#include "osr/cch_preprocessing.h"

#include <iostream>

#include "osr/ways.h"


cch::mip_proc::mip_proc(osr::ways const& w)// std::filesystem::path p, cista::mmap::protection const mode)
  : ways_{w} {}

void cch::mip_proc::test_contraction_order() {
  auto importance_copy_ = ways_.r_->node_importance_;
  importance_copy_.resize(20);
  for (auto const& [i, e] : utl::enumerate(importance_copy_)) {std::cout << e << " " << contr_order_[e] << " " << contr_order_[i] << " " << ways_.r_->node_importance_[osr::node_idx_t{i}] << "\n";}
}

// define the contraction order for the preprocessing here
void cch::mip_proc::build_contraction_order() {
  contr_order_.resize(ways_.n_nodes());
  for (auto const [i, rank] : utl::enumerate(ways_.r_->node_importance_)) {
    contr_order_[rank] = osr::node_idx_t{i};
  }
}

bool cch::mip_proc::is_in(osr::vec<osr::node_idx_t> const& neighbors, osr::node_idx_t const& node) {
  bool is_in_ = false;
  for (auto const& n : neighbors) {is_in_ = is_in_ or node == n;}
  return is_in_;
}

bool cch::mip_proc::check_importance(osr::node_idx_t const& lhs, osr::node_idx_t const& rhs) {
  return ways_.r_->node_importance_[lhs] < ways_.r_->node_importance_[rhs];
}
  

osr::vec<osr::node_idx_t> cch::mip_proc::find_neighbors(osr::node_idx_t const& node) {
  osr::vec<osr::node_idx_t> neighbors_;

  // check for existing neighbors
  auto const& in_ways_ = ways_.r_->node_ways_[node];
  auto const& idx_in_ways_ = ways_.r_->node_in_way_idx_[node];
  if (in_ways_.empty() && idx_in_ways_.empty()) {return neighbors_;}

  // add existing neighbors with higher rank
  for (auto const [idx, way] : utl::zip(idx_in_ways_, in_ways_)) {
    if (idx > 0) {
      auto const& pred_ = ways_.r_->way_nodes_[way][idx - 1];
      if (check_importance(node, pred_) && !is_in(neighbors_, pred_)) {neighbors_.push_back(pred_);}
    }
    if (idx < ways_.r_->way_nodes_.size() - 1) {
      auto const& succ_ = ways_.r_->way_nodes_[way][idx + 1];
      if (check_importance(node, succ_) && !is_in(neighbors_, succ_)) {neighbors_.push_back(succ_);}
    }
  }

  //g_plus_up_.neighbors_[rank] = neighbors_;
  std::cout << "neigbors of node " << node << ": ";
  for (auto const& n : neighbors_){std::cout << n << " ";}
  std::cout << "\n";

  return neighbors_;
}

std::uint32_t cch::mip_proc::find_smallest_neighbor(osr::vec<osr::node_idx_t> const& neighbors) {
  osr::vec<std::uint32_t> neighbor_ranks_;
  for (auto const node : neighbors) {
    neighbor_ranks_.push_back(ways_.r_->node_importance_[node]);
    std::cout << node << ": " << ways_.r_->node_importance_[node] << "\n";
  }
  auto const& min_rank_ = std::ranges::min(neighbor_ranks_);
  std::cout << "neighbor with smallest rank: " << min_rank_ << "\n";
  return min_rank_;
}


void cch::mip_proc::perform_contraction() {
  build_contraction_order();
  elimination_tree_.resize(ways_.n_nodes());
  //osr::vec<edge_idx_t> tmp_edges_;

  contr_order_.resize(5);
  elimination_tree_.resize(5);
  for (auto const [rank, node] : utl::enumerate(contr_order_)) {
    std::cout << "\n" << "=== node " << node << " with rank " << ways_.r_->node_importance_[node] << " ===" << "\n";
    auto const& neighbors_ = find_neighbors(node);

    if (neighbors_.empty()) {
      elimination_tree_[rank] = rank;
      continue;
    }
    
    // for (auto const& n : neighbors_) {
    //   tmp_edges_.push_back(edge_idx_t{node, n, false, })
    // }
    elimination_tree_[rank] = find_smallest_neighbor(neighbors_);
  }

  std::cout << "elimination tree: ";
  for (auto const e : elimination_tree_) {std::cout << e << " ";}
  std::cout << "\n";
}