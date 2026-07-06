#include "osr/cch_preprocessing.h"

#include <iostream>

#include "utl/verify.h"

#include "osr/ways.h"

cch::neighborhood::neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank) 
  : node_{node},
    rank_{rank} {}

void cch::neighborhood::sort_neighbors() {
  if (neighbors_.empty()) {return;}

  //sort by increasing rank
  auto sorting_condition_ = [](auto const& lhs, auto const& rhs) {return std::get<1>(lhs) < std::get<1>(rhs);}; //source: https://stackoverflow.com/questions/23816797/how-does-stdsort-work-for-list-of-pairs#23817006 11.06.2026
  std::sort(neighbors_.begin(), neighbors_.end(), sorting_condition_);

  // filter duplicates
  auto last_s = std::unique(neighbors_.begin(), neighbors_.end());
  neighbors_.erase(last_s, neighbors_.end());
}

void cch::neighborhood::filter_higher_neighbors() {

  if (neighbors_.empty()) {return;}

  neighbors_.erase(std::remove_if(neighbors_.begin(), neighbors_.end(), [this](auto const& n) {
    return std::get<1>(n) <= this->rank_;
  }), neighbors_.end());
}

bool cch::neighborhood::is_neighbor(osr::node_idx_t const& n) {
  if (neighbors_.empty()) {return true;}

  bool b = false;
  for (auto const& node : neighbors_) {b = b or std::get<0>(node) == n;}
  return b;
}

void cch::neighborhood::concatenate(neighborhood const& pred) {
  if (pred.neighbors_.empty()) {return;}
  utl::verify(node_ == std::get<0>(pred.neighbors_[0]), "not lowest higher ranked neighbor {}", std::get<0>(pred.neighbors_[0]));

  for (auto const& n : pred.neighbors_) {
    if (std::get<1>(n) <= rank_ || is_neighbor(std::get<0>(n))) {continue;} // only add shortcuts of the direct connection exists and no shortcuts to lower neighbors and to the node itself
    
    neighbors_.push_back(std::tuple(std::get<0>(n),     // shortcut target node
                        std::get<1>(n),                 // target node rank
                        true,                           // is shortcut
                        pred.node_,                     // shortcut via node
                        std::get<4>(pred.neighbors_[0]),// way idx from start to via
                        std::get<4>(n)));               // way idx from via to target
  }

  return;
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

bool cch::mip_proc::is_in(osr::vec<std::tuple<osr::node_idx_t, std::uint32_t, bool, osr::node_idx_t, osr::way_idx_t, osr::way_idx_t>> const& neighbors, osr::node_idx_t const& node) {
  bool is_in_ = false;
  for (auto const& n : neighbors) {is_in_ = is_in_ or node == std::get<0>(n);}
  return is_in_;
}

bool cch::mip_proc::check_importance(osr::node_idx_t const& lhs, osr::node_idx_t const& rhs) {
  return ways_.r_->node_importance_[lhs] < ways_.r_->node_importance_[rhs];
}

void cch::mip_proc::init_neighborhoods() {
  if (contr_order_.empty()) {return;}

  //init the neighborhoods from the initial osr graph
  for (auto const [rank, node] : utl::enumerate(contr_order_)) {
    all_neighbors_.push_back(neighborhood{node, static_cast<std::uint32_t>(rank)});
    
    // check for existing neighbors
    auto const& in_ways_ = ways_.r_->node_ways_[all_neighbors_[rank].node_];
    auto const& idx_in_ways_ = ways_.r_->node_in_way_idx_[all_neighbors_[rank].node_];
    if (in_ways_.empty() && idx_in_ways_.empty()) {continue;}

    // add existing neighbors with higher rank
    for (auto const [idx, way] : utl::zip(idx_in_ways_, in_ways_)) {
      if (idx > 0) {
        auto const& pred_ = ways_.r_->way_nodes_[way][idx - 1];
        if (check_importance(all_neighbors_[rank].node_, pred_) && !is_in(all_neighbors_[rank].neighbors_, pred_)) {
          all_neighbors_[rank].neighbors_.push_back(std::tuple(pred_, ways_.r_->node_importance_[pred_], false, pred_, way, way));
        }
      }
      if (idx < ways_.r_->way_nodes_.size() - 1) {
        auto const& succ_ = ways_.r_->way_nodes_[way][idx + 1];
        if (check_importance(all_neighbors_[rank].node_, succ_) && !is_in(all_neighbors_[rank].neighbors_, succ_)) {
          all_neighbors_[rank].neighbors_.push_back(std::tuple(succ_, ways_.r_->node_importance_[succ_], false, succ_, way, way));
        }
      }
    }
  }

  return;
}

void cch::mip_proc::contract_nodes() {
  // contract the neighbors by adding neighborhood to least higher neighbor
  for (auto n : all_neighbors_) {
    if (n.neighbors_.empty()) {
      //elimination_tree_.push_back(static_cast<std::uint32_t>(n.rank_));
      continue;
    }

    n.sort_neighbors();
    auto& next = all_neighbors_[std::get<1>(n.neighbors_[0])];
    
    next.concatenate(n);
    //elimination_tree_.push_back(static_cast<std::uint32_t>(next.rank_));
  }

  return;
}

void cch::mip_proc::write_shortcuts(cista::mmap::protection mode) {
  auto shortcut_counter = 0;
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

  for (auto n : all_neighbors_) {

    for (auto neighbor : n.neighbors_) {
      if (!std::get<2>(neighbor)) {
        continue;
      }
      node_shortcuts_up[n.node_].push_back(osr::shortcut_idx_t{shortcut_counter});
      node_shortcuts_down[std::get<0>(neighbor)].push_back(osr::shortcut_idx_t{shortcut_counter});
      ++shortcut_counter;
      shortcut_vec.emplace_back(cch::shortcut_properties{.lower_end_ = n.node_, 
                                 .via_ = std::get<3>(neighbor),
                                 .upper_end_ = std::get<0>(neighbor),
                                 .lower_via_ = std::get<4>(neighbor),
                                 .via_upper_ = std::get<5>(neighbor),
                                 .costs_up_ = osr::kInfeasible,
                                 .costs_down_ = osr::kInfeasible});
    }
  }

  for (auto const x : node_shortcuts_up) {
    ways_.r_->node_shortcuts_up_.emplace_back(x);
  }

  ways_.r_->shortcut_properties_.resize(shortcut_vec.size());
  for (auto const [i, sc] : utl::enumerate(shortcut_vec)) {
    ways_.r_->shortcut_properties_[osr::shortcut_idx_t{i}] = sc;
  }

  auto e = std::error_code{};
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_up_data.bin", e);
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_up_index.bin", e);
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_down_data.bin", e);
  std::filesystem::remove(ways_.p_ / "tmp_node_shortcuts_down_index.bin", e);
}

// void cch::mip_proc::basic_customization() {
//   if (ways_.r_->shortcut_properties_.size == 0) {
//     std::cout << "No shortcuts found for customization.\n";
//     return;
//   }

//   for (auto const n : all_neighbors_) {

//   }
// }