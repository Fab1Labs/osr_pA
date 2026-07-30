#include "osr/cch_preprocessing.h"

#include <iostream>
#include <vector>

#include "utl/verify.h"

#include "osr/ways.h"

cch::neighborhood::neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank) 
  : node_{node},
    rank_{rank} {}

void cch::neighborhood::sort_neighbors(osr::vec<neighbor>& nvec) {
  if (neighbors_.size() < 2) {return;}

  //sort by increasing rank
  //source for sorting condition: 
  // https://stackoverflow.com/questions/23816797/how-does-stdsort-work-for-list-of-pairs#23817006 (11.06.2026)
  auto sorting_condition = [&nvec](auto const& lhs, auto const& rhs) {
    return nvec[lhs].rank_ < nvec[rhs].rank_;
  };
  std::sort(neighbors_.begin(), neighbors_.end(), sorting_condition);
  
  // filter duplicates
  auto last_s = std::unique(neighbors_.begin(), neighbors_.end());
  neighbors_.erase(last_s, neighbors_.end());

  for (std::size_t i = 0; i < (neighbors_.size() - 2); ++i) {
    utl::verify(nvec[neighbors_[i]].rank_ <= nvec[neighbors_[i + 1]].rank_,
                "Neighbors are not sorted correctly for node {} with rank {}",
                node_, rank_);
  }
}


cch::mip_proc::mip_proc(osr::ways& w)
  : ways_{w},
    max_neighbors_{0} {}

// define the contraction order for the preprocessing here
void cch::mip_proc::build_contraction_order() {
  contr_order_.resize(ways_.r_->node_properties_.size());
  for (auto const [i, rank] : utl::enumerate(ways_.r_->node_importance_)) {
    contr_order_[rank] = osr::node_idx_t{i};
  }
}

bool cch::mip_proc::is_in(osr::vec<osr::neighbor_idx_t>& neighbors,
                          osr::node_idx_t const& node) {
  bool is_in = false;
  for (auto const& n : neighbors) {
    is_in = is_in or node == all_neighbors_[n].neighbor_;
  }
  return is_in;
}

bool cch::mip_proc::is_neighbor(neighborhood const& nhood, osr::node_idx_t const& n) {
  if (nhood.neighbors_.empty()) {return false;}

  bool b = false;
  //for (auto const& node : neighbors_) {b = b or std::get<0>(node) == n;}
  for (auto const& node : nhood.neighbors_) {
    b = b or all_neighbors_[node].neighbor_ == n;
  }
  return b;
}

bool cch::mip_proc::check_importance(osr::node_idx_t const& lhs, 
                                     osr::node_idx_t const& rhs) {
  return ways_.r_->node_importance_[lhs] < ways_.r_->node_importance_[rhs];
}

// customize access functions to allow shortcuts for more profiles:
bool cch::mip_proc::accessible_way(osr::way_idx_t const& w, osr::direction const& d) {
  auto const& wp = ways_.r_->way_properties_[w];
  return wp.is_car_accessible() && (d == osr::direction::kForward || !wp.is_oneway_car());
}

bool cch::mip_proc::accessible_node(osr::node_idx_t const& n) {
  auto const& np = ways_.r_->node_properties_[n];
  return np.is_car_accessible();
}

void cch::mip_proc::init_neighborhoods() {
  if (contr_order_.empty()) { 
    return;
  }

  //init the neighborhoods from the initial osr graph
  for (auto const [rank, node] : utl::enumerate(contr_order_)) {
    neighborhoods_.push_back(neighborhood{node, static_cast<std::uint32_t>(rank)});
    if (!accessible_node(node)) {
      continue;
    }

    utl::verify(neighborhoods_[rank].node_ == node && neighborhoods_[rank].rank_ == rank,
                "neighborhood initialized incorrectly.\nExpected node: {} instead: {}\nExpected rank: {} instead {}",
                node, neighborhoods_[rank].node_, rank, neighborhoods_[rank].rank_);

    // check for existing neighbors
    auto const& in_ways = ways_.r_->node_ways_[node];
    auto const& idx_in_ways = ways_.r_->node_in_way_idx_[node];
    if (in_ways.empty() && idx_in_ways.empty()) {
      continue;
    }

    // add existing neighbors with higher rank
    for (auto const [idx, way] : utl::zip(idx_in_ways, in_ways)) {
      auto const& wp = ways_.r_->way_properties_[way];
      if (idx > 0 && wp.is_car_accessible()) {
        auto const& pred = ways_.r_->way_nodes_[way][idx - 1];
        if (check_importance(node, pred) && accessible_node(pred)){ // <= füge nun alle möglichkeiten von pred hinzu nicht nur die erste
            // !is_in(neighborhoods_[rank].neighbors_, pred)) {
          neighborhoods_[rank].neighbors_.push_back(all_neighbors_.size());
          all_neighbors_.push_back(neighbor{
            .neighbor_ = pred,
            .rank_ = ways_.r_->node_importance_[pred],
            .via_ = osr::node_idx_t{0U},
            .to_via_id_ = 0,
            .to_neighbor_id_ = 0,
            .edge_ = way,
            .in_way_idx_ = static_cast<std::uint16_t>(idx - 1),
            .dir_ = osr::direction::kBackward,
            .go_up_ = !wp.is_oneway_car(),
            .go_dwn_ = true});
        }
      }
      if (idx < (ways_.r_->way_nodes_[way].size() - 1) && wp.is_car_accessible()) {
        auto const& succ = ways_.r_->way_nodes_[way][idx + 1];
        if (check_importance(node, succ) && accessible_node(succ)){
            // !is_in(neighborhoods_[rank].neighbors_, succ)) {
          neighborhoods_[rank].neighbors_.push_back(all_neighbors_.size());
          all_neighbors_.push_back(neighbor{
            .neighbor_ = succ,
            .rank_ = ways_.r_->node_importance_[succ],
            .via_ = osr::node_idx_t{0U},
            .to_via_id_ = 0,
            .to_neighbor_id_ = 0,
            .edge_ = way,
            .in_way_idx_ = static_cast<std::uint16_t>(idx),
            .dir_ = osr::direction::kForward,
            .go_up_ = true,
            .go_dwn_ = !wp.is_oneway_car()});
        }
      }
    }
  }
}

void cch::mip_proc::concatenate_neighbors(neighborhood const& pred, neighborhood& succ) {
  if (pred.neighbors_.empty()) {
    return;
  }

  auto const& to_pred_struct = all_neighbors_[pred.neighbors_[0]];
  utl::verify(succ.node_ == all_neighbors_[pred.neighbors_[0]].neighbor_, 
              "node {} is not lowest higher ranked neighbor of {}, expected {} at neighborid {}", 
              all_neighbors_[pred.neighbors_[0]].neighbor_,
              pred.node_, succ.node_, pred.neighbors_[0]);
  for (auto const n : pred.neighbors_) {
    auto const& n_struct = all_neighbors_[n];
    if (n_struct.rank_ <= succ.rank_){
      continue;
    }
    auto const go_up = to_pred_struct.go_dwn_ && n_struct.go_up_;
    auto const go_dwn = to_pred_struct.go_up_ && n_struct.go_dwn_;
    if (!go_up && !go_dwn) {
      continue;
    }
    succ.neighbors_.push_back(all_neighbors_.size());
    all_neighbors_.push_back(neighbor{
      .neighbor_ = n_struct.neighbor_,
      .rank_ = n_struct.rank_,
      .via_ = pred.node_,
      .to_via_id_ = pred.neighbors_[0],
      .to_neighbor_id_ = n,
      .edge_ = osr::way_idx_t{0U},
      .in_way_idx_ = 0,
      .dir_ = osr::direction::kForward,
      .go_up_ = go_up, 
      .go_dwn_ = go_dwn});
  }
}

void cch::mip_proc::contract_nodes() {
  // contract the neighbors by adding neighborhood to least higher neighbor
  for (auto n : neighborhoods_) {
    if (n.neighbors_.empty()) {
      //elimination_tree_.push_back(static_cast<std::uint32_t>(n.rank_));
      continue;
    }
    n.sort_neighbors(all_neighbors_);
    if (max_neighbors_ < n.neighbors_.size()) {
      max_neighbors_ = n.neighbors_.size();
    }
    auto const& succ_rank = all_neighbors_[n.neighbors_[0]].rank_;
    auto& next = neighborhoods_[succ_rank];
    concatenate_neighbors(n, next);
    //elimination_tree_.push_back(static_cast<std::uint32_t>(next.rank_));
  }
  //std::cout << "max neighbors: " << max_neighbors_;
}

bool cch::mip_proc::is_shortcut(neighbor const& n) {
  return n.via_ != osr::node_idx_t{0U} &&
         n.to_via_id_ != 0 &&
         n.to_neighbor_id_ != 0;
}

void cch::mip_proc::filter_neighborhoods() {
  ways_.r_->contraction_order_ = contr_order_;
  ways_.r_->sc_targets_.resize(contr_order_.size());
  for (auto [rank, nh] : utl::enumerate(neighborhoods_)) {
    for (auto sh : nh.neighbors_) {
      auto const target = all_neighbors_[sh].neighbor_;
      ways_.r_->sc_targets_[rank].push_back(target);
    }
  }
  for (std::uint32_t rank = 0; rank < contr_order_.size(); ++rank) {
    if (ways_.r_->sc_targets_[rank].size() < 2) { continue; }
    auto sorting_condition = [this](osr::node_idx_t const lhs, osr::node_idx_t const rhs) {
      return ways_.r_->node_importance_[lhs] < ways_.r_->node_importance_[rhs];
    };
    // sort the given neighbors
    std::sort(ways_.r_->sc_targets_[rank].begin(), ways_.r_->sc_targets_[rank].end(), sorting_condition);
    // filter duplicates
    auto last_s = std::unique(ways_.r_->sc_targets_[rank].begin(), ways_.r_->sc_targets_[rank].end());
    ways_.r_->sc_targets_[rank].erase(last_s, ways_.r_->sc_targets_[rank].end());

    if (ways_.r_->sc_targets_[rank].empty()) { continue; }

    for (std::size_t i = 0; i < (ways_.r_->sc_targets_[rank].size() - 1); ++i) {
      utl::verify(ways_.r_->node_importance_[ways_.r_->sc_targets_[rank][i]] < ways_.r_->node_importance_[ways_.r_->sc_targets_[rank][i + 1]],
                  "Neighbors are sorted incorrectly");
    }
    utl::verify(rank < ways_.r_->node_importance_[ways_.r_->sc_targets_[rank][0]], 
                "Got false lowest higher neighbor of rank {} at next with rank {}", 
                ways_.r_->node_importance_[ways_.r_->sc_targets_[rank][0]], rank);
    
    if (rank == 175) {
      std::cout << "After preprocessing neighborhood of Node with rank 175: ";
      for (auto const n : ways_.r_->sc_targets_[rank]) {
        std::cout << to_idx(ways_.node_to_osm_[n]) << ", ";
      }
      std::cout << "\n";
    }
  }
}

// ./build/osr-extract -i ./test/aachen.osm.pbf -o ./test/aachen

cch::contraction::contraction(cista::wrapped<osr::ways::routing>& r)
  : r_{r} {}

// calculate the neighborhood of all nodes in the graph
void cch::contraction::init_neighborhoods() {
  if (r_->contraction_order_.empty()) {
    return;
  }
  r_->sc_targets_.resize(r_->contraction_order_.size());
  for (auto const [rank, node] : utl::enumerate(r_->contraction_order_)) {
    utl::verify(rank == r_->node_importance_[node], 
                "Expected Node {} with rank {} but node came at rank {}",
                node, r_->node_importance_[node], rank);
    // if (!accessible_node(node)) {
    //   continue;
    // }

    if (rank == 175) {
      std::cout << "Initial neighborhood of Node with rank 63: ";
      for (auto const n : r_->sc_targets_[rank]) {
        std::cout << to_idx(n) << ", ";
      }
      std::cout << "\n";
    } 
    auto const& in_ways = r_->node_ways_[node];
    auto const& idx_in_ways = r_->node_in_way_idx_[node];
    if (in_ways.empty() && idx_in_ways.empty()) {
      continue;
    }

    for (auto const [idx, way] : utl::zip(idx_in_ways, in_ways)) {
      // add neighbors in forward direction with higher rank
      //auto const& wp = r_->way_properties_[way];
      if (idx > 0 ){ //&& wp.is_car_accessible()) {
        auto const& pred = r_->way_nodes_[way][idx - 1];
        if (rank < r_->node_importance_[pred] && accessible_node(pred)) {
          r_->sc_targets_[rank].push_back(pred);
        }
      }
      // add neighbors in backward direction with higher rank
      if (idx < (r_->way_nodes_[way].size() - 1)){ // && wp.is_car_accessible()) {
        auto const& succ = r_->way_nodes_[way][idx + 1];
        if (rank < r_->node_importance_[succ] && accessible_node(succ)) {
          r_->sc_targets_[rank].push_back(succ);
        }
      }

      if (rank == 175) {
        std::cout << "Initial neighborhood of Node with rank 63: ";
        for (auto const n : r_->sc_targets_[rank]) {
          std::cout << to_idx(n) << ", ";
        }
        std::cout << "\n";
      } 
    }
  }
}

// contract the nodes sorted by rank
void cch::contraction::contract_nodes() {
  if (r_->sc_targets_.empty()) {
    return;
  }

  for (std::uint32_t rank = 0; rank < r_->contraction_order_.size(); ++rank) {
    if (r_->sc_targets_[rank].size() == 0) { continue; }
    sort_and_filter_neighbors(rank);
    auto const& neighbors = r_->sc_targets_[rank];
    auto const& next_rank = r_->node_importance_[neighbors[0]];
    for (auto const& n : neighbors) {
      if (r_->node_importance_[n] > next_rank) {
        r_->sc_targets_[next_rank].push_back(n);
      }
    } 
  }
}