#include "osr/cch_preprocessing.h"

#include <iostream>

#include "utl/verify.h"

#include "osr/ways.h"

cch::neighborhood::neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank) 
  : node_{node},
    rank_{rank} {}

void cch::neighborhood::sort_neighbors(osr::vec<neighbor>& nvec) {
  if (neighbors_.size() < 2) {return;}

  //sort by increasing rank
  //source: https://stackoverflow.com/questions/23816797/how-does-stdsort-work-for-list-of-pairs#23817006 11.06.2026
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
  : ways_{w} {}

// define the contraction order for the preprocessing here
void cch::mip_proc::build_contraction_order() {
  contr_order_.resize(ways_.n_nodes());
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

void cch::mip_proc::init_neighborhoods() {
  if (contr_order_.empty()) { 
    return;
  }

  //init the neighborhoods from the initial osr graph
  for (auto const [rank, node] : utl::enumerate(contr_order_)) {
    neighborhoods_.push_back(neighborhood{node, static_cast<std::uint32_t>(rank)});

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
      if (idx > 0) {
        auto const& pred = ways_.r_->way_nodes_[way][idx - 1];
        if (check_importance(node, pred)){ // <= füge nun alle möglichkeiten von pred hinzu nicht nur die erste
            //!is_in(neighborhoods_[rank].neighbors_, pred)) {
          neighborhoods_[rank].neighbors_.push_back(all_neighbors_.size());
          all_neighbors_.push_back(neighbor{
            .neighbor_ = pred,
            .rank_ = ways_.r_->node_importance_[pred],
            .via_ = osr::node_idx_t{0U},
            .to_via_id_ = 0,
            .to_neighbor_id_ = 0,
            .edge_ = way,
            .in_way_idx_ = static_cast<std::uint16_t>(idx - 1),
            .dir_ = osr::direction::kBackward});
        }
      }
      if (idx < ways_.r_->way_nodes_.size() - 1) {
        auto const& succ = ways_.r_->way_nodes_[way][idx + 1];
        if (check_importance(node, succ)){
            //!is_in(neighborhoods_[rank].neighbors_, succ)) {
          neighborhoods_[rank].neighbors_.push_back(all_neighbors_.size());
          all_neighbors_.push_back(neighbor{
            .neighbor_ = succ,
            .rank_ = ways_.r_->node_importance_[succ],
            .via_ = osr::node_idx_t{0U},
            .to_via_id_ = 0,
            .to_neighbor_id_ = 0,
            .edge_ = way,
            .in_way_idx_ = static_cast<std::uint16_t>(idx),
            .dir_ = osr::direction::kForward});
        }
      }
    }
  }
}

void cch::mip_proc::concatenate_neighbors(neighborhood const& pred, neighborhood& succ) {
  if (pred.neighbors_.empty()) {
    return;
  }
  //utl::verify(node_ == std::get<0>(pred.neighbors_[0]), "not lowest higher ranked neighbor {}", std::get<0>(pred.neighbors_[0]));
  utl::verify(succ.node_ == all_neighbors_[pred.neighbors_[0]].neighbor_, 
              "node {} is not lowest higher ranked neighbor of {}, expected {} at neighborid {}", 
              all_neighbors_[pred.neighbors_[0]].neighbor_,
              pred.node_, succ.node_, pred.neighbors_[0]);
  for (auto const n : pred.neighbors_) {
    auto const n_struct = all_neighbors_[n];
    if (n_struct.rank_ <= succ.rank_){ // <= nehme is_neighbor raus, um später optimalen shortcut zu finden
    //if (n_struct.rank_ <= succ.rank_ || is_neighbor(succ, n_struct.neighbor_)) { 
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
      .dir_ = osr::direction::kForward});
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
    auto const succ_rank = all_neighbors_[n.neighbors_[0]].rank_;
    auto& next = neighborhoods_[succ_rank];
    concatenate_neighbors(n, next);
    //elimination_tree_.push_back(static_cast<std::uint32_t>(next.rank_));
  }
}

bool cch::mip_proc::is_shortcut(neighbor const& n) {
  return n.via_ != osr::node_idx_t{0U} &&
         n.to_via_id_ != 0 &&
         n.to_neighbor_id_ != 0;
}

void cch::mip_proc::write_shortcuts(cista::mmap::protection mode) {
  std::vector<cch::shortcut_properties> shortcut_vec;

  auto node_shortcuts_up = osr::mm_paged_vecvec<osr::node_idx_t, osr::shortcut_idx_t>{
    cista::paged<osr::mm_vec32<osr::shortcut_idx_t>>{
        osr::mm_vec32<osr::shortcut_idx_t>{mm("tmp_node_shortcuts_up_data.bin", mode)}},
    osr::mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
        mm("tmp_node_shortcuts_up_index.bin", mode)}};

  auto node_shortcuts_down = osr::mm_paged_vecvec<osr::node_idx_t, osr::shortcut_idx_t>{
    cista::paged<osr::mm_vec32<osr::shortcut_idx_t>>{
        osr::mm_vec32<osr::shortcut_idx_t>{mm("tmp_node_shortcuts_down_data.bin", mode)}},
    osr::mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
        mm("tmp_node_shortcuts_down_index.bin", mode)}};

  node_shortcuts_up.resize(ways_.node_to_osm_.size());
  node_shortcuts_down.resize(ways_.node_to_osm_.size());

  for (auto n : neighborhoods_) {
    for (auto neighbor : n.neighbors_) {
      node_shortcuts_up[n.node_].push_back(static_cast<osr::shortcut_idx_t>(neighbor));
      node_shortcuts_down[all_neighbors_[neighbor].neighbor_].push_back(static_cast<osr::shortcut_idx_t>(neighbor));
    }
  }

  for (auto const x : node_shortcuts_up) {
    ways_.r_->node_shortcuts_up_.emplace_back(x);
  }
  for (auto const x : node_shortcuts_down) {
    ways_.r_->node_shortcuts_down_.emplace_back(x);
  }

  ways_.r_->shortcut_properties_.resize(all_neighbors_.size());
  ways_.r_->shortcut_cost_car_fw_.resize(all_neighbors_.size());
  ways_.r_->shortcut_cost_car_bw_.resize(all_neighbors_.size());
  for (auto const [i, neighbor] : utl::enumerate(all_neighbors_)) {
    ways_.r_->shortcut_properties_[osr::shortcut_idx_t{i}] = shortcut_properties{
        .via_ = all_neighbors_[i].via_,
        .lower_via_ = osr::shortcut_idx_t{all_neighbors_[i].to_via_id_},
        .via_upper_ = osr::shortcut_idx_t{all_neighbors_[i].to_neighbor_id_},
        .edge_ = all_neighbors_[i].edge_,
        .dir_ = all_neighbors_[i].dir_,
        .in_way_idx_ = all_neighbors_[i].in_way_idx_,
      };
  }

  auto e = std::error_code{};
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_up_data.bin", e);
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_up_index.bin", e);
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_down_data.bin", e);
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_down_index.bin", e);
}