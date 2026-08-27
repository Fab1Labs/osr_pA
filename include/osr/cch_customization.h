#pragma once

#include <iostream>

#include "utl/verify.h"
#include "utl/enumerate.h"
#include "utl/zip.h"

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
      "Risk of Segmentation Fault! In_ways.size() = {}, In_way_idx.size() = {}",
      in_ways.size(), in_way_idx.size());

    if (in_ways.empty() && in_way_idx.empty()) {
      return way_data{.way_ = osr::way_idx_t::invalid(), 
                      .dir_ = osr::direction::kBackward, 
                      .way_pos_ = 0};
    }
    for (auto const [idx, way] : utl::zip(in_way_idx, in_ways)) {
      auto const& wp = r_->way_properties_[way];
      if (!wp.is_car_accessible_) { continue; }
      if (idx > 0 && r_->way_nodes_[way][idx - 1] == to) {
        return way_data{.way_ = way, 
                        .dir_ = osr::direction::kBackward, 
                        .way_pos_ = static_cast<std::uint16_t>(idx - 1)};
      }
      if (idx < (r_->way_nodes_[way].size() - 1) && r_->way_nodes_[way][idx + 1] == to) {
        return way_data{.way_ = way, 
                        .dir_ = osr::direction::kForward, 
                        .way_pos_ = static_cast<std::uint16_t>(idx)};
      }
    }
    return way_data{.way_ = osr::way_idx_t::invalid(), 
                    .dir_ = osr::direction::kForward, 
                    .way_pos_ = 0};
  }

  template<osr::direction SearchDir, bool WithRestrictions, bool IsBus>
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
                  "Restriction test: Expected {} but got {}", via, sc_down.nodes_.back());
      if (r_->is_restricted<SearchDir, IsBus>(via, 
                                   r_->get_way_pos(via, sc_down.ways_.back()), 
                                   r_->get_way_pos(via, sc_up.ways_.back()))) {
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
    utl::verify(r_->contraction_order_.size() == r_->sc_targets_.size(), 
                "Risk of Segmentation Fault! Contraction order ({}) is not same size as neighborhood ({})",
                r_->contraction_order_.size(), r_->sc_targets_.size());
    for (auto [rank, n] : utl::enumerate(r_->sc_targets_)) {
      if (n.empty()) { continue; }
      auto const& node = r_->contraction_order_[rank];
      auto const node_cost = osr::clamp_cost(static_cast<std::uint64_t>(P::node_cost(params, r_->node_properties_[node])));
      r_->sc_costs_up_[rank].resize(n.size());
      r_->sc_costs_down_[rank].resize(n.size());
      r_->sc_up_[rank].resize(n.size());
      r_->sc_down_[rank].resize(n.size());
      utl::verify(n.size() == r_->sc_costs_up_[rank].size(),
                  "Risk of Segmentation Fault! Neighborhood size: {}, sc up size: {}",
                  n.size(), r_->sc_costs_up_[rank].size());

      for (auto const [idx, neighbor] : utl::enumerate(n)) {
        utl::verify(r_->node_importance_[node] < r_->node_importance_[neighbor], 
            "Invalid Shortcut: Node {} -> Neighbor {}", r_->node_importance_[node], r_->node_importance_[neighbor]);
        auto const wd = find_way(node, neighbor);
        if (wd.way_ == osr::way_idx_t::invalid()) {
          r_->sc_costs_up_[rank][idx] = osr::kInfeasible;
          r_->sc_costs_down_[rank][idx] = osr::kInfeasible;

          r_->sc_up_[rank][idx] = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
          r_->sc_up_[rank][idx].add(neighbor, wd.way_, wd.dir_, osr::kInfeasible);

          r_->sc_down_[rank][idx] = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
          r_->sc_down_[rank][idx].add(node, wd.way_, osr::opposite(wd.dir_), osr::kInfeasible);
        } else {
          auto const& wp = r_->way_properties_[wd.way_];
          auto const dist = r_->get_way_node_distance(wd.way_, wd.way_pos_);

          auto const wc_up = P::way_cost(params, wp, wd.dir_, dist);
          auto const wc_down = P::way_cost(params, wp, osr::opposite(wd.dir_), dist);
          auto const neighbor_cost = P::node_cost(params, r_->node_properties_[neighbor]);
          auto cost_up = osr::clamp_cost(static_cast<std::uint64_t>(wc_up)) +
                         osr::clamp_cost(static_cast<std::uint64_t>(neighbor_cost));
          auto cost_down = osr::clamp_cost(static_cast<std::uint64_t>(wc_down)) + 
                           osr::clamp_cost(static_cast<std::uint64_t>(node_cost));

          if (wc_up == osr::kInfeasible || neighbor_cost == osr::kInfeasible) {
            cost_up = osr::kInfeasible;
          }
          if (wc_down == osr::kInfeasible || node_cost == osr::kInfeasible) {
            cost_down = osr::kInfeasible;
          }     
          r_->sc_costs_up_[rank][idx] = cost_up;
          r_->sc_costs_down_[rank][idx] = cost_down;

          r_->sc_up_[rank][idx] = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
          r_->sc_up_[rank][idx].add(neighbor, wd.way_, wd.dir_, cost_up);

          r_->sc_down_[rank][idx] = sc_properties{.nodes_ = {}, .ways_ = {}, .dirs_ = {}, .costs_ = {}};
          r_->sc_down_[rank][idx].add(node, wd.way_, osr::opposite(wd.dir_), cost_down);

          utl::verify(r_->sc_down_[rank][idx].nodes_.back() == node && 
                      r_->sc_down_[rank][idx].get_path_cost() == cost_down,
                      "Downward Edge is not initialized correctly.");
          utl::verify(r_->sc_up_[rank][idx].nodes_.back() == neighbor &&
                      r_->sc_up_[rank][idx].get_path_cost() == cost_up,
                      "Upward Edge is not initialized correctly.");
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

  template<bool WithRestrictions, bool isBus>
  void basic_customization() {
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
          auto const new_cost_up = node_to_neighbor_cost_down + r_->sc_costs_up_[rank][t_idx];
          // utl::verify((new_cost_up < old_cost_up && 
          //              new_cost_up != osr::kInfeasible && 
          //              r_->sc_costs_up_[rank][t_idx] != osr::kInfeasible &&
          //              node_to_neighbor_cost_down != osr::kInfeasible) == (
          //             check_sc_update<WithRestrictions, isBus>(rank, t_idx, old_cost_up, new_cost_up, node_to_neighbor_cost_down, true)),
          //             "Shortcut update condition is different! (up)");
          if (check_sc_update<osr::direction::kForward, WithRestrictions, isBus>(
                  rank, t_idx, n_idx, old_cost_up, new_cost_up,  node_to_neighbor_cost_down, true)) {
            r_->sc_costs_up_[neighbor_rank][t_in_n_idx] = new_cost_up;
            auto new_sc_up = r_->sc_down_[rank][n_idx];
            new_sc_up.append(r_->sc_up_[rank][t_idx]);
            r_->sc_up_[neighbor_rank][t_in_n_idx] = new_sc_up;

            utl::verify(r_->sc_up_[neighbor_rank][t_in_n_idx].ways_.back() != osr::way_idx_t::invalid(),
                        "Got unexpected invalid way in new upward shortcut");
            utl::verify(r_->sc_up_[neighbor_rank][t_in_n_idx].nodes_.back() == target && 
                        r_->sc_up_[neighbor_rank][t_in_n_idx].get_path_cost() == r_->sc_costs_up_[neighbor_rank][t_in_n_idx],
                        "Upward Shortcut is not initialized correctly.");
          }

          auto const& old_cost_down = r_->sc_costs_down_[neighbor_rank][t_in_n_idx];
          auto const new_cost_down = node_to_neighbor_cost_up + r_->sc_costs_down_[rank][t_idx];
          // utl::verify((new_cost_down < old_cost_down &&
          //              new_cost_down != osr::kInfeasible &&
          //              r_->sc_costs_down_[rank][t_idx] != osr::kInfeasible &&
          //              node_to_neighbor_cost_up != osr::kInfeasible) ==
          //             check_sc_update<WithRestrictions, isBus>(rank, t_idx, old_cost_down, new_cost_down, node_to_neighbor_cost_up, false),
          //             "Shortcut update condition is different! (down)");
          if (check_sc_update<osr::direction::kBackward, WithRestrictions, isBus>(
                  rank, t_idx, n_idx, old_cost_down, new_cost_down, node_to_neighbor_cost_up, false)) {
            r_->sc_costs_down_[neighbor_rank][t_in_n_idx] = new_cost_down;
            auto new_sc_down = r_->sc_down_[rank][t_idx];
            new_sc_down.append(r_->sc_up_[rank][n_idx]);
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

  // helper function
  // void validate_costs(osr::vec<osr::cost_t> const& costs, bool is_up) {
  //   auto const msg = " Expected increasing costs on path";
  //   for (std::size_t i = 0; i < (costs.size() - 1); ++i) {
  //     utl::verify(costs[i] <= costs[i + 1],
  //         is_up ? std::string("[CSC UP]") + msg : std::string("[CSC DOWN]") + msg);
  //   }
  // }

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
  
  cista::wrapped<osr::ways::routing>& r_;
};
} // namespace cch