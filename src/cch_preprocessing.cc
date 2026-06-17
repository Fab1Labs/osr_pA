#include "osr/cch_preprocessing.h"

#include <iostream>

#include "osr/ways.h"

cch::neighborhood::neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank) 
  : node_{node},
    rank_{rank} {}

void cch::neighborhood::sort_neighbors() {
  if (neighbors_.empty()) {return;}

  //sort by increasing rank
  auto sorting_condition = [](auto const& lhs, auto const& rhs) {return lhs.second < rhs.second;}; //source: https://stackoverflow.com/questions/23816797/how-does-stdsort-work-for-list-of-pairs#23817006 11.06.2026
  std::sort(neighbors_.begin(), neighbors_.end(), sorting_condition);

  // filter duplicates
  auto last_s = std::unique(neighbors_.begin(), neighbors_.end());
  neighbors_.erase(last_s, neighbors_.end());
}

void cch::neighborhood::filter_higher_neighbors() {

  if (neighbors_.empty()) {return;}

  neighbors_.erase(std::remove_if(neighbors_.begin(), neighbors_.end(), [this](auto const& n) {
    return n.second <= this->rank_;
  }), neighbors_.end());
}

void cch::neighborhood::concatenate(osr::vec<std::pair<osr::node_idx_t, std::uint32_t>> const& new_neighbors) {
  if (new_neighbors.empty()) {return;}

  for (auto const& n : new_neighbors) {
    if (n.second > rank_) {neighbors_.push_back(n);}
  }
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

bool cch::mip_proc::is_in(osr::vec<std::pair<osr::node_idx_t, std::uint32_t>> const& neighbors, osr::node_idx_t const& node) {
  bool is_in_ = false;
  for (auto const& n : neighbors) {is_in_ = is_in_ or node == n.first;}
  return is_in_;
}

bool cch::mip_proc::check_importance(osr::node_idx_t const& lhs, osr::node_idx_t const& rhs) {
  return ways_.r_->node_importance_[lhs] < ways_.r_->node_importance_[rhs];
}

void cch::mip_proc::find_neighbors(cch::neighborhood& neighborhood) {

  // check for existing neighbors
  auto const& in_ways_ = ways_.r_->node_ways_[neighborhood.node_];
  auto const& idx_in_ways_ = ways_.r_->node_in_way_idx_[neighborhood.node_];
  if (in_ways_.empty() && idx_in_ways_.empty()) {return;}

  // add existing neighbors with higher rank
  for (auto const [idx, way] : utl::zip(idx_in_ways_, in_ways_)) {
    if (idx > 0) {
      auto const& pred_ = ways_.r_->way_nodes_[way][idx - 1];
      if (check_importance(neighborhood.node_, pred_) && !is_in(neighborhood.neighbors_, pred_)) {neighborhood.neighbors_.push_back(std::pair(pred_, ways_.r_->node_importance_[pred_]));}
    }
    if (idx < ways_.r_->way_nodes_.size() - 1) {
      auto const& succ_ = ways_.r_->way_nodes_[way][idx + 1];
      if (check_importance(neighborhood.node_, succ_) && !is_in(neighborhood.neighbors_, succ_)) {neighborhood.neighbors_.push_back(std::pair(succ_, ways_.r_->node_importance_[succ_]));}
    }
  }

  return;
}

void cch::mip_proc::init_neighborhoods() {
  if (contr_order_.empty()) {return;}

  //init the neighborhoods from the initial osr graph
  for (auto const [rank, node] : utl::enumerate(contr_order_)) {
    all_neighbors_.push_back(neighborhood{node, static_cast<std::uint32_t>(rank)});
    find_neighbors(all_neighbors_[rank]);
  }

  // contract the neighbors by adding neighborhood to least higher neighbor
  for (auto& n : all_neighbors_) {
    if (n.neighbors_.empty()) {
      elimination_tree_.push_back(static_cast<std::uint32_t>(n.rank_));
      continue;}

    n.sort_neighbors();
    auto& next = all_neighbors_[n.neighbors_[0].second];
    next.concatenate(n.neighbors_);
    elimination_tree_.push_back(static_cast<std::uint32_t>(next.rank_));
  }

  //for (std::size_t i = 0; i < 5; ++i) {std::cout << "node " << all_neighbors_[i].node_ << " with smallest neighbor " << all_neighbors_[i].neighbors_[0].second << "\n";}
  return;
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