#pragma once

#include "iostream"

#include "utl/verify.h"
#include "utl/enumerate.h"

#include "osr/routing/profiles/car.h"
#include "osr/routing/with_profile.h"
#include "osr/cch_preprocessing.h"
#include "osr/ways.h"
#include "osr/types.h"

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
    auto const& via = prep_.all_neighbors_[nidx].via_;
    auto const& to_via_idx = prep_.all_neighbors_[nidx].to_via_id_;
    auto const& to_neighbor_idx = prep_.all_neighbors_[nidx].to_neighbor_id_;
    if (via == osr::node_idx_t{0U} && 
        to_via_idx == 0 &&
        to_neighbor_idx == 0) {
      auto const& way = prep_.all_neighbors_[nidx].edge_;
      auto const& dir = prep_.all_neighbors_[nidx].dir_;
      way_in_neighbor_[nidx] = way;
      dir_in_neighbor_[nidx] = dir;
      way_out_neighbor_[nidx] = way;
      dir_out_neighbor_[nidx] = dir;

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
      cch::neighborhood const& curr, osr::neighbor_idx_t nidx, neighbor const& next) {
    utl::verify(curr.rank_ <= next.rank_, "illegal neighborhood with {} > {}", curr.rank_, next.rank_);

    // handle direct neighbors
    if (next.to_via_id_ == 0 && next.to_neighbor_id_ == 0) {
      auto const wp = ways_.r_->way_properties_[next.edge_];
      auto const dist = ways_.r_->get_way_node_distance(next.edge_, next.in_way_idx_);

      // calculate upward costs:
      auto const upper_nc = P::node_cost(params, ways_.r_->node_properties_[next.neighbor_]);
      auto const wc_up = P::way_cost(params, wp, next.dir_, dist);
      //std::cout << "way cost up: " << wc_up << " with dist: " << dist << " wp: " << wp.max_speed_s_per_m() << " + nc: " << upper_nc << "\n";

      if (upper_nc == osr::kInfeasible || wc_up == osr::kInfeasible) {
        neighbor_costs_up_[nidx] = osr::kInfeasible;
      } else {
        neighbor_costs_up_[nidx] = osr::clamp_cost(static_cast<std::uint64_t>(wc_up)) + upper_nc;
      }
      // calculate downward costs:
      auto const lower_nc = P::node_cost(params, ways_.r_->node_properties_[curr.node_]);
      auto const wc_down = P::way_cost(params, wp, osr::opposite(next.dir_), dist);

      if (lower_nc == osr::kInfeasible || wc_down == osr::kInfeasible) {
        neighbor_costs_down_[nidx] = osr::kInfeasible;
      } else {
        neighbor_costs_down_[nidx] = osr::clamp_cost(static_cast<std::uint64_t>(wc_down)) + lower_nc;
      }
    } else { // handle concatenated neighbor (shortcut)
      //  calculate upward costs: 
      auto const& to_via_c_up = neighbor_costs_down_[next.to_via_id_];
      auto const& to_neighbor_c_up = neighbor_costs_up_[next.to_neighbor_id_];

      auto const& from_lower_to_via = way_in_neighbor_[next.to_via_id_];
      auto const& from_via_to_upper = way_in_neighbor_[next.to_neighbor_id_];

      auto const is_u_turn = from_lower_to_via == from_via_to_upper;

      if (to_via_c_up == osr::kInfeasible || to_neighbor_c_up == osr::kInfeasible) {
        neighbor_costs_up_[nidx] = osr::kInfeasible;
      } else {
        auto const turn_angle_up = ways_.r_->get_turn_angle(
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

  void customize(cch::neighborhood const& node) {
    if (node.neighbors_.empty()) {
      return;
    }

    auto const find_shortest = [&](osr::vec<osr::neighbor_idx_t> const neighbors, 
                                   std::size_t idx, osr::cost_t const neighbor_costs,
                                   osr::vec<osr::cost_t> all_costs) {
      auto const& curr_struct = prep_.all_neighbors_[neighbors[idx]];
      for (auto i = idx + 1; i < neighbors.size(); ++i) {
        auto const& next_struct = prep_.all_neighbors_[neighbors[i]];
        if (!(curr_struct.neighbor_ == next_struct.neighbor_ && 
            curr_struct.via_ == next_struct.neighbor_)) {
          continue;
        }
        if (all_costs[neighbors[i]] >= neighbor_costs) {
          all_costs[neighbors[i]] = osr::kInfeasible;
        } else {
          all_costs[neighbors[idx]] = osr::kInfeasible;
          break;
        }
      }
    };

    for (auto const [idx, neighbor] : utl::enumerate(node.neighbors_)) {
      auto const& costs_up = neighbor_costs_up_[neighbor];
      auto const& costs_down = neighbor_costs_down_[neighbor];
      if (costs_up != osr::kInfeasible) {
        find_shortest(node.neighbors_, idx, costs_up, neighbor_costs_up_);
      }
      if (costs_down != osr::kInfeasible) {
        find_shortest(node.neighbors_, idx, costs_down, neighbor_costs_down_);
      }
    }
  }

  template <typename Fn>
  auto with_valid_profile(osr::search_profile const p, Fn&& fn) {
    if (p == osr::search_profile::kCar) {
      return fn(osr::car{});
    }
    if (p == osr::search_profile::kBus) {
      return fn(osr::bus{});
    }

    throw utl::fail("cch customization is not implemented for profile {}.", to_str(p));
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
      for (auto const neighbor : node.neighbors_) {
        get_turn_data(neighbor);
        get_neighbor_cost<P>(params, node, neighbor, prep_.all_neighbors_[neighbor]);
      }
    }

    // filter all unreachable shortcuts:
    shortcut_translation_.resize(prep_.all_neighbors_.size());
    ways_.r_->node_shortcuts_up_.resize(ways_.node_to_osm_.size());
    ways_.r_->node_shortcuts_down_.resize(ways_.node_to_osm_.size());
    for (auto node : prep_.neighborhoods_) {
      for (auto& neighbor : node.neighbors_) {
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
      }
    }
    // for (auto const& node : prep_.neighborhoods_) {
    //   customize(node);
    // }
    return;
  }

  void run(osr::search_profile const& profile, osr::profile_parameters const& params) {
    return with_valid_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return run<P>(pp);
    });
  }

  void export_costs() {
    if (neighbor_costs_up_.empty() && neighbor_costs_down_.empty()) {
      return;
    }
    for (auto const [id, entry] : utl::enumerate(neighbor_costs_up_)) {
      ways_.r_->shortcut_cost_car_up_[osr::shortcut_idx_t{id}] = entry;
    }
    for (auto const [id, entry] : utl::enumerate(neighbor_costs_down_)) {
      ways_.r_->shortcut_cost_car_down_[osr::shortcut_idx_t{id}] = entry;
    }
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


} // namespace cch