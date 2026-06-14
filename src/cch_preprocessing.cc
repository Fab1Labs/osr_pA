#include "osr/cch_preprocessing.h"

#include <iostream>

#include "osr/ways.h"

cch::neighborhood::neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank) 
  : node_{node},
    rank_{rank} {}

void cch::neighborhood::add_neighbors(osr::vec<osr::node_idx_t> const& neighbors, osr::vec_map<osr::node_idx_t, std::uint32_t> const& ranks) {
  if (neighbors.empty()) {return;}
  for (auto const& n : neighbors) {
    neighbors_.push_back(std::pair(n, ranks[n]));
  }
}

void cch::neighborhood::sort_neighbors() {
  if (neighbors_.empty()) {return;}
  //sort by increasing rank 
  std::sort(neighbors_.begin(), neighbors_.end(), [](auto const& lhs, auto const& rhs) {
    return lhs.second < rhs.second; // source: https://stackoverflow.com/questions/23816797/how-does-stdsort-work-for-list-of-pairs#23817006 11.06.2026
  });
}

void cch::neighborhood::filter_higher_neighbors() {

  if (neighbors_.empty()) {return;}

  neighbors_.erase(std::remove_if(neighbors_.begin(), neighbors_.end(), [this](auto const& n) {
    return n.second <= this->rank_;
  }), neighbors_.end());
}

void cch::neighborhood::concatenate_neighbors(neighborhood target) {
  if (neighbors_.empty()) {return;}

  for (auto const& n : neighbors_) {target.neighbors_.push_back(n);}
}

cch::mip_proc::mip_proc(osr::ways const& w)
  : ways_{w} {}

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

  return neighbors_;
}

void cch::mip_proc::init_neighborhoods() {
  if (contr_order_.empty()) {return;}

  //init the neighborhoods from the initial osr graph
  for (auto const& [rank, node] : utl::enumerate(contr_order_)) {
    all_neighbors_.push_back(neighborhood{node, static_cast<std::uint32_t>(rank)});
    all_neighbors_[rank].add_neighbors(find_neighbors(node), ways_.r_->node_importance_);
  }
}

// void cch::mip_proc::perform_contraction() {
//   build_contraction_order();
//   elimination_tree_.resize(ways_.n_nodes());

//   contr_order_.resize(12);
//   elimination_tree_.resize(12);

//   // init all neighborhoods
//   for (auto const& [rank, node] : utl::enumerate(contr_order_)) {
//     //std::cout << "\n" << "=== node " << node << " with rank " << rank << " ===" << "\n";
//     all_neighbors_.push_back(neighborhood{node, static_cast<std::uint32_t>(rank)});
//     auto const& neighbors_ = find_neighbors(node);
//     for (auto const n : neighbors_) {all_neighbors_[rank].add_neighbor(n, ways_.r_->node_importance_[n]);}

//     if (neighbors_.empty()) {
//       elimination_tree_[rank] = rank;
//     } else {
//       elimination_tree_[rank] = all_neighbors_[rank].neighbors_[0].second;
//     }
//   }

//   // contract
//   for (auto& curr : all_neighbors_) {
//     curr.sort_neighbors();
//     if (curr.neighbors_.empty()) {continue;}

//     curr.filter_higher_neighbors();

//     auto const& next_rank_ = curr.neighbors_[0].second;
//     curr.concatenate_neighbors(all_neighbors_[next_rank_]);

//     //std::cout << "new neighborhood of node " << curr.node_ << "\n";
//     //for (auto const n : curr.neighbors_) {std::cout << "neighbor node: " << n.first << ", rank: " << n.second << "\n";}

//   }
  
//   // std::cout << "elimination tree: ";
//   // for (auto const e : elimination_tree_) {std::cout << e << " ";}
//   // std::cout << "\n";
// }