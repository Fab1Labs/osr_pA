#pragma once

#include  <iostream>

#include "utl/verify.h"
#include "utl/enumerate.h"

#include "osr/routing/profiles/car.h"
#include "osr/routing/with_profile.h"
#include "osr/cch_preprocessing.h"
#include "osr/ways.h"
#include "osr/types.h"
#include "osr/shortcut.h"

namespace cch {

struct basic_customization {

  using turn_data_t = std::tuple<osr::way_idx_t, std::uint16_t, osr::way_idx_t, std::uint16_t>;
  // falls unerklärliche Ergebnisse: Additional nodes und blocked nicht beachtet bis jetzt

  basic_customization(osr::ways& w, mip_proc& p)
    : ways_(w), 
      prep_(p) {}

  osr::vec<osr::neighbor_idx_t> check_appearences(osr::node_idx_t const& target, 
                                                  osr::vec<osr::neighbor_idx_t> const& neighbors) {
    osr::vec<osr::neighbor_idx_t> target_edges;
    for (auto const n : neighbors) {
      if (prep_.all_neighbors_[n].neighbor_ == target) {
        target_edges.push_back(n);
      }
    }
    return target_edges;
  }

  void get_turn_data(osr::neighbor_idx_t const& nidx) {
    // calculate the first and last way idx of the shortcut
    auto const& n_struct = prep_.all_neighbors_[nidx];
    auto const& to_via_idx = prep_.all_neighbors_[nidx].to_via_id_;
    auto const& to_neighbor_idx = prep_.all_neighbors_[nidx].to_neighbor_id_;
    if (n_struct.via_ == osr::node_idx_t{0U} && 
        to_via_idx == 0 &&
        to_neighbor_idx == 0) {
      way_in_neighbor_[nidx] = n_struct.edge_;
      dir_in_neighbor_[nidx] = n_struct.dir_;
      way_out_neighbor_[nidx] = n_struct.edge_;
      dir_out_neighbor_[nidx] = n_struct.dir_;

    } else {
      utl::verify(to_via_idx <= nidx || to_neighbor_idx <= nidx,
                  "Neighbor of id {} has lower neighbors {}, {} as a shortcut",
                  nidx, to_via_idx, to_neighbor_idx);
      way_in_neighbor_[nidx] = way_out_neighbor_[to_via_idx];
      dir_in_neighbor_[nidx] = osr::opposite(dir_out_neighbor_[to_via_idx]); 
      way_out_neighbor_[nidx] = way_out_neighbor_[to_neighbor_idx];
      dir_out_neighbor_[nidx] = dir_out_neighbor_[to_neighbor_idx];
    }
  }

  template<osr::Profile P>
  void get_neighbor_cost(typename P::parameters const& params,  
      cch::neighborhood const& curr, osr::neighbor_idx_t const& nidx, neighbor const& next) {
    utl::verify(curr.rank_ <= next.rank_, "illegal neighborhood with {} > {}", curr.rank_, next.rank_);

    // handle direct neighbors
    if (next.to_via_id_ == 0 && next.to_neighbor_id_ == 0) {
      auto const& wp = ways_.r_->way_properties_[next.edge_];
      auto const dist = ways_.r_->get_way_node_distance(next.edge_, next.in_way_idx_);

      // calculate upward costs:
      auto const upper_nc = P::node_cost(params, ways_.r_->node_properties_[next.neighbor_]);
      auto const wc_up = P::way_cost(params, wp, next.dir_, dist);
      //std::cout << "way cost up: " << wc_up << " with dist: " << dist << " wp: " << wp.max_speed_s_per_m() << " + nc: " << upper_nc << "\n";

      if (upper_nc == osr::kInfeasible || wc_up == osr::kInfeasible) {
        neighbor_costs_up_[nidx] = osr::kInfeasible;
      } else {
        neighbor_costs_up_[nidx] = osr::clamp_cost(static_cast<std::uint64_t>(wc_up + upper_nc));
      }
      // calculate downward costs:
      auto const lower_nc = P::node_cost(params, ways_.r_->node_properties_[curr.node_]);
      auto const wc_down = P::way_cost(params, wp, osr::opposite(next.dir_), dist);

      if (lower_nc == osr::kInfeasible || wc_down == osr::kInfeasible) {
        neighbor_costs_down_[nidx] = osr::kInfeasible;
      } else {
        neighbor_costs_down_[nidx] = osr::clamp_cost(static_cast<std::uint64_t>(wc_down + lower_nc));
      }
    } else { // handle concatenated neighbors (shortcut)
      // calculate upward costs: 
      auto const& to_via_c_up = neighbor_costs_down_[next.to_via_id_];
      auto const& to_neighbor_c_up = neighbor_costs_up_[next.to_neighbor_id_];

      auto const& from_lower_to_via = way_in_neighbor_[next.to_via_id_];
      auto const& from_via_to_upper = way_in_neighbor_[next.to_neighbor_id_];

      auto const is_u_turn = from_lower_to_via == from_via_to_upper && 
                             dir_in_neighbor_[next.to_via_id_] != dir_in_neighbor_[next.to_neighbor_id_];

      if (to_via_c_up == osr::kInfeasible || to_neighbor_c_up == osr::kInfeasible) {
        neighbor_costs_up_[nidx] = osr::kInfeasible;
      } else {
        auto const turn_angle_up = ways_.r_->get_turn_angle( // HIER BITTE NOCHMAL GENAU WEGEN DEN RICHTUNGEN SCHAUEN
            next.via_,
            ways_.r_->get_way_pos(next.via_, from_lower_to_via),
            dir_in_neighbor_[next.to_via_id_],
            ways_.r_->get_way_pos(next.via_, from_via_to_upper),
            dir_in_neighbor_[next.to_neighbor_id_]);
        auto const via_turn_c_up = P::turn_cost(params, turn_angle_up);
        neighbor_costs_up_[nidx] = to_via_c_up + 
                                   via_turn_c_up + 
                                   to_neighbor_c_up + (
                                   is_u_turn ? params.uturn_penalty_ : 0U);
      }
      // calculate downward costs:
      auto const& to_via_c_down = neighbor_costs_up_[next.to_via_id_];
      auto const& to_neighbor_c_down = neighbor_costs_down_[next.to_neighbor_id_];

      if (to_via_c_down == osr::kInfeasible || to_neighbor_c_down == osr::kInfeasible) {
        neighbor_costs_down_[nidx] = osr::kInfeasible;
      } else {
        auto const turn_angle_down = ways_.r_->get_turn_angle(
            next.via_,
            ways_.r_->get_way_pos(next.via_, from_via_to_upper),
            dir_in_neighbor_[next.to_neighbor_id_],
            ways_.r_->get_way_pos(next.via_, from_lower_to_via),
            dir_in_neighbor_[next.to_via_id_]);
        auto const turn_angle_c_down = P::turn_cost(params, turn_angle_down);
        neighbor_costs_down_[nidx] = to_via_c_down + 
                                     turn_angle_c_down + 
                                     to_neighbor_c_down + 
                                     (is_u_turn ? params.uturn_penalty_ : 0U);    
      }
    }
  }

  template<osr::Profile P>
  void run(typename P::parameters const& params) {
    auto const neighbor_size = prep_.all_neighbors_.size();
    neighbor_costs_up_.resize(neighbor_size);
    neighbor_costs_down_.resize(neighbor_size);
    way_in_neighbor_.resize(neighbor_size);
    way_out_neighbor_.resize(neighbor_size);
    dir_in_neighbor_.resize(neighbor_size);
    dir_out_neighbor_.resize(neighbor_size);

    // calculate all costs for each shortcut 
    for (auto const& node : prep_.neighborhoods_) {
      for (auto const& neighbor : node.neighbors_) {
        get_turn_data(neighbor);
        get_neighbor_cost<P>(params, node, neighbor, prep_.all_neighbors_[neighbor]);
      }
    }

    // filter all unreachable shortcuts:
    shortcut_translation_.resize(prep_.all_neighbors_.size());
    ways_.r_->node_shortcuts_up_.resize(ways_.node_to_osm_.size());
    ways_.r_->node_shortcuts_down_.resize(ways_.node_to_osm_.size());
    for (auto const& node : prep_.neighborhoods_) {
      for (auto const& neighbor : node.neighbors_) {
        // skip unreachable shortcuts in both ways:
        if (neighbor_costs_up_[neighbor] == osr::kInfeasible &&
            neighbor_costs_down_[neighbor] == osr::kInfeasible) {
          continue;
        }
        // add the neighbor to the valid shortcuts and store new idx
        auto const& neighbor_struct = prep_.all_neighbors_[neighbor];
        auto shortcut_idx = osr::shortcut_idx_t{ways_.r_->shortcut_properties_.size()};
        shortcut_translation_[neighbor] = shortcut_idx;
        ways_.r_->node_shortcuts_up_[node.node_].push_back(shortcut_idx);
        ways_.r_->node_shortcuts_down_[neighbor_struct.neighbor_].push_back(shortcut_idx);
        ways_.r_->shortcut_properties_.push_back(shortcut_properties{
          .lower_end_ = node.node_,
          .upper_end_ = neighbor_struct.neighbor_,
          .via_ = neighbor_struct.via_,
          .lower_via_ = osr::shortcut_idx_t{
            shortcut_translation_[neighbor_struct.to_via_id_]},
          .via_upper_ = osr::shortcut_idx_t{
            shortcut_translation_[neighbor_struct.to_neighbor_id_]},
          .edge_ = neighbor_struct.edge_,
          .dir_ = neighbor_struct.dir_,
          .in_way_idx_ = neighbor_struct.in_way_idx_}
        );
        ways_.r_->shortcut_costs_up_.push_back(neighbor_costs_up_[neighbor]);
        ways_.r_->shortcut_costs_down_.push_back(neighbor_costs_down_[neighbor]);
        ways_.r_->in_shortcut_.push_back(cch::shortcut_nav_infos{
          .way_ = way_in_neighbor_[neighbor],
          .dir_ = dir_in_neighbor_[neighbor]
        });
        ways_.r_->out_shortcut_.push_back(cch::shortcut_nav_infos{
          .way_ = way_out_neighbor_[neighbor],
          .dir_ = dir_out_neighbor_[neighbor]
        });
      }
    }

    // compare shortcuts with same lower-, via- and upper node and set the higher costs to infeasible
    // auto const& find_shortest = [&](auto const shortcuts, std::size_t const idx,
    //                                 osr::cost_t const sc_cost, 
    //                                 osr::vec<osr::cost_t> all_costs) {
    //   auto const& curr_struct = ways_.r_->shortcut_properties_[shortcuts[idx]];
    //   for (auto i = idx + 1; i < shortcuts.size(); ++i) {
    //     auto const& next_struct = ways_.r_->shortcut_properties_[shortcuts[i]];
    //     // filter direct connections and sc via different nodes here:
    //     if (!(curr_struct.upper_end_ == next_struct.upper_end_ &&
    //           curr_struct.via_ == next_struct.via_)) {
    //       continue;
    //     }
    //     // only change costs, if they are not equal:
    //     if (all_costs[shortcuts[i]] > sc_cost) {
    //       all_costs[shortcuts[i]] = osr::kInfeasible;
    //       continue;
    //     }
    //     if (all_costs[shortcuts[i] < sc_cost]) {
    //       all_costs[shortcuts[idx]] = osr::kInfeasible;
    //       break;
    //     }
    //   }
    // };

    // customize the given shortcuts here:
    // for (auto const& n : ways_.r_->node_shortcuts_up_) {
    //   for (auto const [idx, sc] : utl::enumerate(n)) {
    //     auto const& cost_up = ways_.r_->shortcut_costs_up_[sc];
    //     auto const& cost_dwn = ways_.r_->shortcut_costs_down_[sc];
    //     if (cost_up != osr::kInfeasible) {
    //       find_shortest(n, idx, cost_up, ways_.r_->shortcut_costs_up_);
    //     }
    //     if(cost_dwn != osr::kInfeasible) {
    //       find_shortest(n, idx, cost_dwn, ways_.r_->shortcut_costs_down_);
    //     }
    //   }
    // }
    return;
  }

  void run(osr::search_profile const& profile, osr::profile_parameters const& params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return run<P>(pp);
    });
  }

  osr::ways& ways_;
  mip_proc& prep_;
  osr::vec<osr::shortcut_idx_t> shortcut_translation_;
  osr::vec<cch::neighborhood> profile_nodes_;
  osr::vec<osr::cost_t> neighbor_costs_up_;
  osr::vec<osr::cost_t> neighbor_costs_down_;
  osr::vec<osr::way_idx_t> way_in_neighbor_; // store way idx of lower end
  osr::vec<osr::way_idx_t> way_out_neighbor_;// store way idx of higher end
  osr::vec<osr::direction> dir_in_neighbor_;
  osr::vec<osr::direction> dir_out_neighbor_;
};

struct customization {

  struct way_data {
    osr::way_idx_t way_;
    osr::direction dir_;
    std::uint16_t way_pos_;
  };

  customization(cista::wrapped<osr::ways::routing>& r)
  : r_{r} {}

  void calculate_direct_costs(osr::search_profile const& profile,
                              osr::profile_parameters const& params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return calculate_direct_costs<P>(pp);
    });
  }

  // helper function to find way, dir and pos of two neighbors
  way_data find_way(osr::node_idx_t const& from, osr::node_idx_t const& to) {
    auto const& in_ways = r_->node_ways_[from];
    auto const& in_way_idx = r_->node_in_way_idx_[from];
    utl::verify(in_ways.size() == in_way_idx.size(),
      "Risk of Segmentation Fautl! In_ways.size() = {}, In_way_idx.size() = {}",
      in_ways.size(), in_way_idx.size());
    if (in_ways.empty() && in_way_idx.empty()) {
      return way_data{.way_ = osr::way_idx_t::invalid(), 
                      .dir_ = osr::direction::kForward, 
                      .way_pos_ = 0};
    }
    for (auto const [idx, way] : utl::zip(in_way_idx, in_ways)) {
      if (idx > 0) {
        if (r_->way_nodes_[way][idx - 1] == to) {
          return way_data{.way_ = way, 
                          .dir_ = osr::direction::kBackward, 
                          .way_pos_ = static_cast<std::uint16_t>(idx - 1)};
        }
      }
      if (idx < in_ways.size() - 1) {
        if (r_->way_nodes_[way][idx + 1] == to) {
          return way_data{.way_ = way, 
                          .dir_ = osr::direction::kForward, 
                          .way_pos_ = static_cast<std::uint16_t>(idx)};
        }
      }
    }
    return way_data{.way_ = osr::way_idx_t::invalid(), 
                    .dir_ = osr::direction::kForward, 
                    .way_pos_ = 0};
  }

  // calculate the existing edge costs here for customization preparation
  template<osr::Profile P>
  void calculate_direct_costs(typename P::parameters const& params) {
    auto const size = r_->contraction_order_.size();
    r_->sc_costs_up_.resize(size);
    r_->sc_costs_down_.resize(size);
    utl::verify(r_->contraction_order_.size() == r_->sc_targets_.size(), 
                "Risk of Segmentation Fault! Contraction order ({}) is not same size as neighborhood ({})",
                r_->contraction_order_.size(), r_->sc_targets_.size());
    for (auto [rank, n] : utl::enumerate(r_->sc_targets_)) {
      if (n.empty()) { continue; }
      auto const& node = r_->contraction_order_[rank];
      auto const node_cost = osr::clamp_cost(P::node_cost(params, r_->node_properties_[node]));
      r_->sc_costs_up_[rank].resize(n.size());
      r_->sc_costs_down_[rank].resize(n.size());
      utl::verify(n.size() == r_->sc_costs_up_[rank].size(),
                  "Risk of Segmentation Fault! Neighborhood size: {}, sc up size: {}",
                  n.size(), r_->sc_costs_up_[rank].size());

      for (auto const [idx, neighbor] : utl::enumerate(n)) {
        utl::verify(r_->node_importance_[node] < r_->node_importance_[neighbor], 
            "Invalid Shortcut: Node {} -> Neighbor {}", r_->node_importance_[node], r_->node_importance_[neighbor]);
        auto const wd = find_way(node, neighbor);
        if (wd.way_ == osr::way_idx_t::invalid()) {
          r_->sc_costs_up_[rank][idx] = osr::clamp_cost(osr::kInfeasible);
          r_->sc_costs_down_[rank][idx] = osr::clamp_cost(osr::kInfeasible);
        } else {
          auto const& wp = r_->way_properties_[wd.way_];
          auto const dist = r_->get_way_node_distance(wd.way_, wd.way_pos_);
          r_->sc_costs_up_[rank][idx] = osr::clamp_cost(P::way_cost(params, wp, wd.dir_, dist)) +
                                        osr::clamp_cost(P::node_cost(params, r_->node_properties_[neighbor]));
          r_->sc_costs_down_[rank][idx] = osr::clamp_cost(P::way_cost(params, wp, osr::opposite(wd.dir_), dist)) +
                                          node_cost;
        }
      }
    }
  }

  std::size_t find_target(osr::vec<osr::node_idx_t> const& targets, osr::node_idx_t t) { 
    for (auto const [i, n] : utl::enumerate(targets)) {
      if (n == t) {
        return i;
      }
    }
    return targets.size();
  }

  void basic_customization() {
    for (std::uint32_t rank = 0; rank < r_->contraction_order_.size(); ++rank) {
      utl::verify(r_->node_importance_.size() == r_->contraction_order_.size(), "rank provoked SF");
      auto const& current_neighbors = r_->sc_targets_[rank];
      if (current_neighbors.empty()) {
        continue;
      }
      // customize for the edge from node to neighbor
      for (auto const [n_idx, neighbor] : utl::enumerate(current_neighbors)) {
        auto const& neighbor_rank = r_->node_importance_[neighbor];
        utl::verify(rank < neighbor_rank, "node rank {} > neighbor rank {}!!", rank, neighbor_rank);
        auto const& targets = r_->sc_targets_[neighbor_rank];
        auto const& uv_cost_up = r_->sc_costs_up_[rank][n_idx];
        auto const& uv_cost_down = r_->sc_costs_down_[rank][n_idx];
        
        for (std::size_t t_idx = n_idx + 1; t_idx < current_neighbors.size(); ++t_idx) {
          auto const& target = current_neighbors[t_idx];
          auto const t_in_n_idx = find_target(targets, target);
          if (t_in_n_idx == targets.size()) { 
            continue; }
          r_->sc_costs_up_[neighbor_rank][t_in_n_idx] = std::min(r_->sc_costs_up_[neighbor_rank][t_in_n_idx], 
                                                                 uv_cost_down + r_->sc_costs_up_[rank][t_idx]);
          r_->sc_costs_down_[neighbor_rank][t_in_n_idx] = std::min(r_->sc_costs_down_[neighbor_rank][t_in_n_idx],
                                                                   uv_cost_up + r_->sc_costs_down_[rank][t_idx]);
        }
      }
    }
  }
  
  cista::wrapped<osr::ways::routing>& r_;
};
} // namespace cch