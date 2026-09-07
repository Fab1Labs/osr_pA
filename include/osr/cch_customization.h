#pragma once

#include <iostream>

#include <utility>

#include "osr/routing/profiles/car.h"
#include "osr/routing/with_profile.h"
#include "osr/cch_preprocessing.h"
#include "osr/ways.h"
#include "osr/types.h"
#include "osr/shortcut.h"

namespace cch {

struct customization {

  struct way_data {
    osr::way_idx_t way_;
    osr::direction dir_;
    std::uint16_t node_in_way_idx_;
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

  void get_cch_edges(osr::search_profile const& profile,
                     osr::profile_parameters const& params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return get_cch_edges<P>(pp);
    });
  }

  template<bool WithRestrictions, bool IsBus>
  void customize_shortcuts(osr::search_profile const& profile,
                           osr::profile_parameters const& params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return customize_shortcuts<P, WithRestrictions, IsBus>(pp);
    });
  }

  template<bool WithRestrictions, bool IsBus>
  void basic_customization(osr::search_profile const& profile,
                           osr::profile_parameters const params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return basic_customization<P, WithRestrictions, IsBus>(pp);
    });
  }

  template<osr::Profile P>
  void get_cch_edges(typename P::parameters const& params) {
    r_->cch_cost_up_.resize(r_->contraction_order_.size());
    r_->cch_cost_down_.resize(r_->contraction_order_.size());
    r_->cch_sc_up_.resize(r_->contraction_order_.size());
    r_->cch_sc_down_.resize(r_->contraction_order_.size());

    for (auto const [rank, node] : utl::enumerate(r_->contraction_order_)) {
      r_->cch_cost_up_[rank].resize(r_->sc_targets_[rank].size(), osr::kInfeasible);
      r_->cch_cost_down_[rank].resize(r_->sc_targets_[rank].size(), osr::kInfeasible);
      r_->cch_sc_up_[rank].resize(r_->sc_targets_[rank].size(), packed_shortcut::invalid());
      r_->cch_sc_down_[rank].resize(r_->sc_targets_[rank].size(), packed_shortcut::invalid());

      auto const node_cost = P::node_cost(params, r_->node_properties_[node]);
      if (node_cost == osr::kInfeasible) {
        continue;
      }

      for (auto const [way, idx] : 
           utl::zip(r_->node_ways_[node], r_->node_in_way_idx_[node])) {
        auto const get_edge = [&](osr::direction const dir, std::uint16_t const from,
                                  std::uint16_t const to) {
          // check importance and accessibility
          auto const neighbor = r_->way_nodes_[way][to];
          auto const neighbor_p = r_->node_properties_[neighbor];
          if (r_->node_importance_[neighbor] <= rank ||
              !neighbor_p.is_car_accessible_) {
            return;
          }

          auto const wp = r_->way_properties_[way];
          auto const target_idx = r_->get_target_idx(node, neighbor);
          auto const dist = r_->get_way_node_distance(way, std::min(from, to));

          // check way cost up
          if (P::way_cost(params, wp, dir, 0U) != osr::kInfeasible &&
              P::node_cost(params, neighbor_p) != osr::kInfeasible) {
            r_->cch_cost_up_[rank][target_idx] = P::way_cost(params, wp, dir, dist) +
                                                 P::node_cost(params, neighbor_p);
          }

          // check way cost down
          if (P::way_cost(params, wp, osr::opposite(dir), 0U) != osr::kInfeasible &&
              node_cost != osr::kInfeasible) {
            r_->cch_cost_down_[rank][target_idx] = P::way_cost(params, wp, osr::opposite(dir), dist) +
                                                   node_cost;
          }

          auto const upper = cch::target_node{neighbor, r_->get_way_pos(neighbor, way, to), dir};
          auto const lower = cch::target_node{node, r_->get_way_pos(node, way, from), osr::opposite(dir)};
          // add "shortcuts" of edge length 1 upward
          if (r_->cch_cost_up_[rank][target_idx] != osr::kInfeasible) {
            r_->cch_sc_up_[rank][target_idx] = cch::packed_shortcut{
              .entry_node_ = lower,
              .exit_node_ = upper,
              .down_ = 0U,
              .up_ = 0U,
              .via_rank_ = 0U,
              .u_turn_penalty_ = osr::cost_t{0U}
            };
          }
          // add "shortcuts" of edge length 1 downward
          if (r_->cch_cost_down_[rank][target_idx] != osr::kInfeasible) {
            r_->cch_sc_down_[rank][target_idx] = cch::packed_shortcut{
              .entry_node_ = upper,
              .exit_node_ = lower,
              .down_ = 0U,
              .up_ = 0U,
              .via_rank_ = 0U,
              .u_turn_penalty_ = osr::cost_t{0}
            };
          }
        };

        if (idx != 0U) {
          get_edge(osr::direction::kBackward, idx, idx - 1);
        }
        if (idx != r_->way_nodes_[way].size() - 1U) {
          get_edge(osr::direction::kForward, idx, idx + 1);
        }
      }
    }
  }

  template<osr::Profile P, bool WithRestrictions, bool IsBus>
  void customize_shortcuts(typename P::parameters const& params) {
    for (auto const [rank, node] : utl::enumerate(r_->contraction_order_)) {
      auto const& neighbors = r_->sc_targets_[rank];
      utl::verify(neighbors.size() == r_->cch_sc_up_[rank].size() &&
                  neighbors.size() == r_->cch_sc_down_[rank].size() &&
                  neighbors.size() == r_->cch_cost_up_[rank].size() &&
                  neighbors.size() == r_->cch_cost_down_[rank].size(),
                  "[CCH Customization] Unequal array sizes");
      if (neighbors.empty() || !r_->node_properties_[node].is_car_accessible_) {
        continue;
      }
      //shortcut goes entry -> node -> target
      for (auto const [n_idx, entry] : utl::enumerate(neighbors)) {
        auto const& n_rank = r_->node_importance_[entry];
        auto const& targets = r_->sc_targets_[n_rank];
        auto const& node_to_entry_cost = r_->cch_cost_up_[rank][n_idx];
        auto const& entry_to_node_cost = r_->cch_cost_down_[rank][n_idx];

        for (std::size_t t_idx = n_idx + 1; t_idx < neighbors.size(); ++t_idx) {
          auto const& target = neighbors[t_idx];
          utl::verify(r_->node_importance_[target] > r_->node_importance_[entry], 
                      "[CUSTOMIZATION] Expected higher rank from entry to target");
          auto const t_n_idx = find_target(targets, target);

          if (t_n_idx == targets.size()) {
            continue;
          }

          // check for shortcut up
          auto const& entry_to_target_cost = r_->cch_cost_up_[n_rank][t_n_idx];
          auto u_turn_penalty_up = osr::kInfeasible;
          if(entry_to_node_cost != osr::kInfeasible &&
             r_->cch_cost_down_[rank][t_idx] != osr::kInfeasible) {
            u_turn_penalty_up = get_penalty<P, WithRestrictions, IsBus>(params, 
                     node, r_->cch_sc_down_[rank][n_idx], r_->cch_sc_up_[rank][t_idx]);
          }
          
          if (combineable(entry_to_target_cost, r_->cch_cost_up_[rank][t_idx], 
                          entry_to_node_cost, u_turn_penalty_up)) {
            r_->cch_cost_up_[n_rank][t_n_idx] = entry_to_node_cost + u_turn_penalty_up + 
                                                r_->cch_cost_up_[rank][t_idx];
            auto& shortcut_up = r_->cch_sc_up_[n_rank][t_n_idx];
            combine_shortcuts(shortcut_up, rank, n_idx, t_idx, u_turn_penalty_up);
          }

          // check for shortcut down
          auto const& target_to_entry_cost = r_->cch_cost_down_[n_rank][t_n_idx];
          auto u_turn_penalty_down = osr::kInfeasible;
          if (node_to_entry_cost != osr::kInfeasible &&
              r_->cch_cost_down_[rank][t_idx] != osr::kInfeasible) {
            u_turn_penalty_down = get_penalty<P, WithRestrictions, IsBus>(params,
                     node, r_->cch_sc_down_[rank][t_idx], r_->cch_sc_up_[rank][n_idx]);
          }

          if (combineable(target_to_entry_cost, node_to_entry_cost, 
                          r_->cch_cost_down_[rank][t_idx], u_turn_penalty_down)) {
            r_->cch_cost_down_[n_rank][t_n_idx] = r_->cch_cost_down_[rank][t_idx] + 
                                                  u_turn_penalty_down + node_to_entry_cost;
            auto& shortcut_down = r_->cch_sc_down_[n_rank][t_n_idx];
            combine_shortcuts(shortcut_down, rank, t_idx, n_idx,  u_turn_penalty_down);
          }
        }
      }
    }
  }

  template<osr::Profile P, bool WithRestrictions, bool IsBus>
  osr::cost_t get_penalty(typename P::parameters const& params, osr::node_idx_t const& n,
                          packed_shortcut const& down, packed_shortcut const& up) {
    auto const& exit_down = down.exit_node_;
    auto const& entry_up = up.entry_node_;

    utl::verify((exit_down.n_ == n || exit_down.n_ == osr::node_idx_t::invalid()) &&
                 (entry_up.n_ == n || entry_up.n_ == osr::node_idx_t::invalid()),
                 "[GET_PENALTY] Expected same meetpoint at {} but got {} and {}",
                 n, exit_down.n_, entry_up.n_);
    
    if (exit_down.n_ != entry_up.n_) {
      return osr::kInfeasible;
    }

    if constexpr (WithRestrictions) {
      if (r_->is_restricted<osr::direction::kForward, 
                            IsBus>(n, exit_down.way_, entry_up.way_)) {
        return osr::kInfeasible;
      }
    }

    if (exit_down.way_ == entry_up.way_ && 
        exit_down.dir_ == osr::opposite(entry_up.dir_)) {
      return params.uturn_penalty_;
    } else {
      return osr::cost_t{0U};
    }
  }

  bool combineable(osr::cost_t const& old_cost,
                   osr::cost_t const& via_to_target_cost,
                   osr::cost_t const& entry_to_via_cost,
                   osr::cost_t const& penalty) {
    if (penalty == osr::kInfeasible || 
        via_to_target_cost == osr::kInfeasible || 
        entry_to_via_cost == osr::kInfeasible) {
      return false;
    }

    auto const new_cost = entry_to_via_cost + penalty +
                          via_to_target_cost;
    if (old_cost <= new_cost || new_cost == osr::kInfeasible) {
      return false;
    } 

    return true;
  }

  void combine_shortcuts(packed_shortcut& new_shortcut,
                         std::size_t const via_rank, 
                         std::size_t const entry_idx,
                         std::size_t const target_idx, 
                         osr::cost_t const& penalty) {
    auto const& new_entry = r_->cch_sc_down_[via_rank][entry_idx].entry_node_;
    auto const& new_exit = r_->cch_sc_up_[via_rank][target_idx].exit_node_;

    utl::verify(new_entry.valid() &&  new_exit.valid(),
                "[CCH Shortcut Combination] Failed to combine shortcuts due to invalid target nodes");
    new_shortcut.entry_node_ = new_entry;
    new_shortcut.exit_node_ = new_exit;
    new_shortcut.down_ = entry_idx;
    new_shortcut.up_ = target_idx;
    new_shortcut.via_rank_ = via_rank;
    new_shortcut.u_turn_penalty_ = penalty;
  }

  // helper function to find way, dir and pos of two neighbors
  way_data find_way(osr::node_idx_t const& from, osr::node_idx_t const& to) {
    auto const& in_ways = r_->node_ways_[from];
    auto const& in_way_idx = r_->node_in_way_idx_[from];
    utl::verify(in_ways.size() == in_way_idx.size(),
      "Risk of Segmentation Fault! In_ways.size() = {}, In_way_idx.size() = {}",
      in_ways.size(), in_way_idx.size());

    if (in_ways.empty() && in_way_idx.empty()) {
      return way_data{.way_ = osr::way_idx_t::invalid(), 
                      .dir_ = osr::direction::kBackward, 
                      .node_in_way_idx_ = 0};
    }
    for (auto const [idx, way] : utl::zip(in_way_idx, in_ways)) {
      auto const& wp = r_->way_properties_[way];
      if (!wp.is_car_accessible_) { continue; }
      if (idx > 0 && r_->way_nodes_[way][idx - 1] == to) {
        return way_data{.way_ = way, 
                        .dir_ = osr::direction::kBackward, 
                        .node_in_way_idx_ = static_cast<std::uint16_t>(idx - 1)};
      }
      if (idx < (r_->way_nodes_[way].size() - 1) && r_->way_nodes_[way][idx + 1] == to) {
        return way_data{.way_ = way, 
                        .dir_ = osr::direction::kForward, 
                        .node_in_way_idx_ = static_cast<std::uint16_t>(idx)};
      }
    }
    return way_data{.way_ = osr::way_idx_t::invalid(), 
                    .dir_ = osr::direction::kForward, 
                    .node_in_way_idx_ = 0};
  }

  template<bool WithRestrictions, bool IsBus>
  bool check_sc_update(std::size_t const& rank,
                       std::size_t const& t_idx,
                       std::size_t const& n_idx,
                       osr::cost_t const& old_cost,
                       osr::cost_t const& new_cost,
                       osr::cost_t const& node_to_neighbor_cost,
                       bool is_up) {
    if (old_cost <= new_cost || new_cost == osr::kInfeasible) {
      return false;
    }

    if (is_up && (r_->sc_costs_up_[rank][t_idx] == osr::kInfeasible ||
        node_to_neighbor_cost == osr::kInfeasible)) {
      return false;
    }

    if (!is_up && (r_->sc_costs_down_[rank][t_idx] == osr::kInfeasible ||
        node_to_neighbor_cost == osr::kInfeasible)) {
      return false;
    }

    // -> via node, idx der node in from way und idx der node in to way
    if constexpr (WithRestrictions) {
      auto const& via = r_->contraction_order_[rank];
      auto const& sc_down = is_up ? r_->sc_down_[rank][n_idx] : r_->sc_down_[rank][t_idx];
      auto const& sc_up = is_up ? r_->sc_up_[rank][t_idx] : r_->sc_up_[rank][n_idx];
      utl::verify(via == sc_down.nodes_.back(), 
                  "Restriction test (dw): Expected {} but got {}", via, sc_down.nodes_.back());
      if (r_->is_restricted<osr::direction::kForward, IsBus>(via, 
                                   r_->get_way_pos(via, sc_down.ways_.back()), 
                                   r_->get_way_pos(via, sc_up.ways_[0]))) {
        return false;
      }
    }

    return true;
  }

  // calculate the existing (direct) edge costs for customization preparation
  template<osr::Profile P>
  void calculate_direct_costs(typename P::parameters const& params) {
    auto const size = r_->contraction_order_.size();
    r_->sc_costs_up_.resize(size);
    r_->sc_costs_down_.resize(size);
    r_->sc_up_.resize(size);
    r_->sc_down_.resize(size);

    for (auto [rank, n] : utl::enumerate(r_->sc_targets_)) {
      if (n.empty()) { continue; }
      auto const& node = r_->contraction_order_[rank];
      auto const node_cost = P::node_cost(params, r_->node_properties_[node]);
      r_->sc_costs_up_[rank].resize(n.size());
      r_->sc_costs_down_[rank].resize(n.size());
      r_->sc_up_[rank].resize(n.size());
      r_->sc_down_[rank].resize(n.size());
      utl::verify(n.size() == r_->sc_costs_up_[rank].size(),
                  "Risk of Segmentation Fault! Neighborhood size: {}, sc up size: {}",
                  n.size(), r_->sc_costs_up_[rank].size());

      if (node_cost == osr::kInfeasible) {
        for (auto const [idx, neighbor] : utl::enumerate(n)) {
          r_->sc_costs_up_[rank][idx] = osr::kInfeasible;
          r_->sc_up_[rank][idx] = sc_properties::invalid(neighbor);

          r_->sc_costs_down_[rank][idx] = osr::kInfeasible;
          r_->sc_down_[rank][idx] = sc_properties::invalid(node);
        }
        continue;
      }

      for (auto const [idx, neighbor] : utl::enumerate(n)) {
        utl::verify(r_->node_importance_[node] < r_->node_importance_[neighbor], 
            "Invalid Shortcut: Node {} -> Neighbor {}", 
            r_->node_importance_[node], r_->node_importance_[neighbor]);
        auto const wd = find_way(node, neighbor);
        if (wd.way_ == osr::way_idx_t::invalid()) {
          r_->sc_costs_up_[rank][idx] = osr::kInfeasible;
          r_->sc_up_[rank][idx] = sc_properties::invalid(neighbor);

          r_->sc_costs_down_[rank][idx] = osr::kInfeasible;
          r_->sc_down_[rank][idx] = sc_properties::invalid(node);
        } else {
          auto const& wp = r_->way_properties_[wd.way_];
          auto const dist = r_->get_way_node_distance(wd.way_, wd.node_in_way_idx_);

          if (P::way_cost(params, wp, wd.dir_, 0U) != osr::kInfeasible &&
              P::node_cost(params, r_->node_properties_[neighbor]) != osr::kInfeasible) {
            auto const wc_up = P::way_cost(params, wp, wd.dir_, dist);
            auto const neighbor_cost = P::node_cost(params, r_->node_properties_[neighbor]);
            auto cost_up = osr::clamp_cost(static_cast<std::uint64_t>(wc_up)) +
                           osr::clamp_cost(static_cast<std::uint64_t>(neighbor_cost));
            if (wc_up == osr::kInfeasible || neighbor_cost == osr::kInfeasible) {
              cost_up = osr::kInfeasible;
            }

            r_->sc_costs_up_[rank][idx] = cost_up;
            r_->sc_up_[rank][idx] = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
            r_->sc_up_[rank][idx].add(neighbor, wd.way_, wd.dir_, cost_up);

            utl::verify(r_->sc_up_[rank][idx].nodes_.back() == neighbor &&
                        r_->sc_up_[rank][idx].get_path_cost() == cost_up,
                        "Upward Edge is not initialized correctly.");
          } else {
            r_->sc_costs_up_[rank][idx] = osr::kInfeasible;
            r_->sc_up_[rank][idx] = sc_properties::invalid(neighbor);
          }

          if (P::way_cost(params, wp, osr::opposite(wd.dir_), 0U) != osr::kInfeasible &&
              P::node_cost(params, r_->node_properties_[node]) != osr::kInfeasible) {
            auto const wc_down = P::way_cost(params, wp, osr::opposite(wd.dir_), dist);
            auto cost_down = osr::clamp_cost(static_cast<std::uint64_t>(wc_down)) + 
                             osr::clamp_cost(static_cast<std::uint64_t>(node_cost));
            if (wc_down == osr::kInfeasible || node_cost == osr::kInfeasible) {
              cost_down = osr::kInfeasible;
            }

            r_->sc_costs_down_[rank][idx] = cost_down;
            r_->sc_down_[rank][idx] = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
            r_->sc_down_[rank][idx].add(node, wd.way_, osr::opposite(wd.dir_), cost_down);

            utl::verify(r_->sc_down_[rank][idx].nodes_.back() == node && 
                        r_->sc_down_[rank][idx].get_path_cost() == cost_down,
                        "Downward Edge is not initialized correctly.");
          } else {
            r_->sc_costs_down_[rank][idx] = osr::kInfeasible;
            r_->sc_down_[rank][idx] = sc_properties::invalid(node);
          }
        }
      }
    }
  }

  std::size_t find_target(osr::vec<osr::node_idx_t> const& targets, osr::node_idx_t const& t) { 
    for (auto const [i, n] : utl::enumerate(targets)) {
      if (n == t) {
        return i;
      }
    }
    return targets.size();
  }

  template<osr::Profile P>
  osr::cost_t apply_u_turn_penalty(typename P::parameters const& params, 
                                   sc_properties const& down, sc_properties const& up) {
    if (down.ways_.back() == up.ways_[0] && down.dirs_.back() == osr::opposite(up.dirs_[0])) {
      return params.uturn_penalty_;
    } else {
      return osr::cost_t{0U};
    }
  }

  template<osr::Profile P, bool WithRestrictions, bool isBus>
  void basic_customization(typename P::parameters const& params) {
    for (std::uint32_t rank = 0; rank < r_->contraction_order_.size(); ++rank) {
      utl::verify(r_->node_importance_.size() == r_->contraction_order_.size(), 
                  "rank caused segmentation fault");
      auto const& current_neighbors = r_->sc_targets_[rank];
      if (current_neighbors.empty()) {
        continue;
      }
      // customize for the edge from node to neighbor
      for (auto const [n_idx, neighbor] : utl::enumerate(current_neighbors)) {
        auto const& neighbor_rank = r_->node_importance_[neighbor];
        utl::verify(rank < neighbor_rank, "node rank {} > neighbor rank {}!!", rank, neighbor_rank);
        auto const& targets = r_->sc_targets_[neighbor_rank];
        auto const& node_to_neighbor_cost_up = r_->sc_costs_up_[rank][n_idx];
        auto const& node_to_neighbor_cost_down = r_->sc_costs_down_[rank][n_idx];
        
        for (std::size_t t_idx = n_idx + 1; t_idx < current_neighbors.size(); ++t_idx) {
          auto const& target = current_neighbors[t_idx];
          auto const t_in_n_idx = find_target(targets, target);
          if (t_in_n_idx == targets.size()) { 
            continue; 
          }
          auto const& old_cost_up = r_->sc_costs_up_[neighbor_rank][t_in_n_idx];
          auto const u_turn_penalty_up = apply_u_turn_penalty<P>(params, 
                r_->sc_down_[rank][n_idx], r_->sc_up_[rank][t_idx]);
          auto const new_cost_up = node_to_neighbor_cost_down + 
                                        r_->sc_costs_up_[rank][t_idx] + 
                                        u_turn_penalty_up;
          if (check_sc_update<WithRestrictions, isBus>(
                  rank, t_idx, n_idx, old_cost_up, new_cost_up, node_to_neighbor_cost_down, true)) {
            r_->sc_costs_up_[neighbor_rank][t_in_n_idx] = new_cost_up;
            auto new_sc_up = r_->sc_down_[rank][n_idx];
            new_sc_up.append(r_->sc_up_[rank][t_idx], u_turn_penalty_up);
            r_->sc_up_[neighbor_rank][t_in_n_idx] = new_sc_up;

            utl::verify(r_->sc_up_[neighbor_rank][t_in_n_idx].ways_.back() != osr::way_idx_t::invalid(),
                        "Got unexpected invalid way in new upward shortcut");
            utl::verify(r_->sc_up_[neighbor_rank][t_in_n_idx].nodes_.back() == target && 
                        r_->sc_up_[neighbor_rank][t_in_n_idx].get_path_cost() == r_->sc_costs_up_[neighbor_rank][t_in_n_idx],
                        "Upward Shortcut is not initialized correctly.");
          }

          auto const& old_cost_down = r_->sc_costs_down_[neighbor_rank][t_in_n_idx];
          auto const u_turn_penalty_down = apply_u_turn_penalty<P>(params, 
                r_->sc_down_[rank][t_idx],r_->sc_up_[rank][n_idx]);
          auto const new_cost_down = node_to_neighbor_cost_up + 
                                     r_->sc_costs_down_[rank][t_idx] + 
                                     u_turn_penalty_down;
          if (check_sc_update<WithRestrictions, isBus>(
                  rank, t_idx, n_idx, old_cost_down, new_cost_down, node_to_neighbor_cost_up, false)) {
            r_->sc_costs_down_[neighbor_rank][t_in_n_idx] = new_cost_down;
            auto new_sc_down = r_->sc_down_[rank][t_idx];
            new_sc_down.append(r_->sc_up_[rank][n_idx], u_turn_penalty_down);
            r_->sc_down_[neighbor_rank][t_in_n_idx] = new_sc_down;

            utl::verify(r_->sc_down_[neighbor_rank][t_in_n_idx].ways_.back() != osr::way_idx_t::invalid(),
                        "Got unexpected invalid way in new downward shortcut");
            utl::verify(r_->sc_down_[neighbor_rank][t_in_n_idx].nodes_.back() == neighbor && 
                        r_->sc_down_[neighbor_rank][t_in_n_idx].get_path_cost() == r_->sc_costs_down_[neighbor_rank][t_in_n_idx],
                        "Downward Shortcut is not initialized correctly.");
          }
        }
      }
    }
  }

  //transform downward paths that they can be used for upward search
  void transform_downward_paths() {
    for (std::uint32_t rank = 0; rank < r_->contraction_order_.size(); ++rank) {
      std::size_t idx = 0;
      auto const& targets = r_->sc_targets_[rank];
      auto const& costs = r_->sc_costs_down_[rank];
      auto& properties = r_->sc_down_[rank];
      for (auto [target, cost, path] : utl::zip(targets, costs, properties)) {
        utl::verify(r_->contraction_order_[rank] == path.nodes_.back(), 
            "[TF DOWN] Expected {} as end of the down path but got {}.",
            r_->contraction_order_[rank], path.nodes_.back());
        
        path.reverse_path(target);
        std::reverse(path.ways_.begin(), path.ways_.end());
        std::reverse(path.dirs_.begin(), path.dirs_.end());
        std::reverse(path.costs_.begin(), path.costs_.end());
        properties[idx] = path;

        utl::verify(path.nodes_.back() == target, 
            "[TF DOWN] Expected target {} but got {}",
            target, path.nodes_.back());
        utl::verify(path.get_path_cost() == cost,
            "[TF DOWN] Expexted costs {} but got {}",
            cost, path.get_path_cost());
        ++idx;
      }
    }
  }

  //helper function
  void validate_connectivity(osr::ways const& ways, cch::sc_properties const& path, 
      osr::node_idx_t const& node, bool is_up) {
    auto const enter_path_msg = " Node {} is not connected to first path way {}";
    auto const path_msg_in = " Node {} is not connected to incoming way {}";
    auto const path_msg_out = " Node {} is not connected to outgoing way {}";
    //auto const exit_path_msg = " Target node {} is not connected to incoming way {}";

    // check the node and entry to the path
    auto const& node_ways = r_->node_ways_[node];
    auto reachable = false;
    for (auto const w : node_ways) {
      if (w == path.ways_[0]) {
        reachable = true;
        break;
      }
    }
    utl::verify(reachable || path.ways_[0] == osr::way_idx_t::invalid(), 
        is_up ? std::string("[CSC UP]") + enter_path_msg : std::string("[CSC DOWN]") + enter_path_msg,
        ways.node_to_osm_[node], ways.way_osm_idx_[path.ways_[0]]);

    // check the validity of the nodes on the path
    for (std::size_t i = 0; i < path.nodes_.size(); ++i) {
      auto const& node_ways = r_->node_ways_[path.nodes_[i]];
      auto reachable_in = false;
      auto reachable_out = false;
      for (auto const w : node_ways) {
        if (w == path.ways_[i]) {
          reachable_in = true;
        }
        if (i < path.nodes_.size() - 1) {
          if (w == path.ways_[i + 1]) {
            reachable_out = true;
          }
        }
      }
      utl::verify(reachable_in || (path.ways_[i] == osr::way_idx_t::invalid() && path.ways_.size() == 1), 
          is_up ? std::string("[CSC UP]") + path_msg_in : std::string("[CSC DOWN]") + path_msg_in,
          ways.node_to_osm_[path.nodes_[i]], ways.way_osm_idx_[path.ways_[i]]);
      if (i < path.nodes_.size() - 1) {
        utl::verify(reachable_out || (path.ways_[i] == osr::way_idx_t::invalid() && path.ways_.size() == 1),
            is_up ? std::string("[CSC UP]") + path_msg_out : std::string("[CSC] DOWN") + path_msg_out,
            ways.node_to_osm_[path.nodes_[i]], ways.way_osm_idx_[path.ways_[i + 1]]);
      }
    }
  } 

  // check if the shortcuts represent valid connections
  void check_shortcut_correctness(osr::ways const& w) {
    for (auto const [rank, node] : utl::enumerate(r_->contraction_order_)) {
      auto const& targets = r_->sc_targets_[rank];
      auto const& costs_up = r_->sc_costs_up_[rank];
      auto const& costs_down = r_->sc_costs_down_[rank];
      auto const& path_up = r_->sc_up_[rank];
      auto const& path_down = r_->sc_down_[rank];

      // check shortcuts updwards
      for (auto [target, cost, path] : utl::zip(targets, costs_up, path_up)) {
        utl::verify(r_->node_importance_[target] > r_->node_importance_[node],
            "[CSC] Importance of target is not higher than from current node");
        utl::verify(target == path.nodes_.back() && cost == path.get_path_cost(),
            "[CSC UP] Expected node {} with costs {} but got path with target {} and costs {}",
            target, cost, path.nodes_.back(), path.get_path_cost());
        //validate_costs(path.costs_, true);
        validate_connectivity(w, path, node, true);
      }

      // check shortcuts downwards
      for (auto [target, cost, path] : utl::zip(targets, costs_down, path_down)) {
        utl::verify(target == path.nodes_.back() && cost == path.get_path_cost(),
            "[CSC DOWN] Expected node {} with costs {} but got path with target {} and costs {}",
            target, cost, path.nodes_.back(), path.get_path_cost());

        // for (std::size_t i = 0; i < (path.costs_.size() - 1); ++i) {
        //   utl::verify(path.costs_[i] <= path.costs_[i + 1],
        //       "[CSC DOWN] Expected increasing costs on path");
        // }
        //validate_costs(path.costs_, false);
        validate_connectivity(w, path, node, false);
      }
    }
  }

  void validate_neighbors(osr::ways const& w) {
    for (auto const [rank, node] : utl::enumerate(r_->contraction_order_)) {
      for (auto [t_idx, target] : utl::enumerate(r_->sc_targets_[rank])) {
        if (r_->sc_costs_up_[rank][t_idx] == osr::kInfeasible) {
          continue;
        }

        auto const& sc_old = r_->sc_up_[rank][t_idx];
        auto const& sc_new = r_->cch_sc_up_[rank][t_idx];

        auto const new_way = r_->node_ways_[sc_new.entry_node_.n_][sc_new.entry_node_.way_];
        utl::verify(new_way == sc_old.ways_.back(), 
                    "[SC UP] Expected way {} but got {} between {} and {}",
                    w.way_osm_idx_[sc_old.ways_.back()], w.way_osm_idx_[new_way], 
                    w.node_to_osm_[sc_new.entry_node_.n_], w.node_to_osm_[sc_new.exit_node_.n_]);
      }
    }
  }
  
  cista::wrapped<osr::ways::routing>& r_;
};
} // namespace cch