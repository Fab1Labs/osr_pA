#pragma once

#include "utl/verify.h"

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

  void get_turn_data(osr::neighbor_idx_t const& nidx) {
    // calculate the first and last way idx of the shortcut
    auto const& via = prep_.all_neighbors_[nidx].via_;
    if (via == osr::node_idx_t{0U} && 
        prep_.all_neighbors_[nidx].to_via_id_ == 0 &&
        prep_.all_neighbors_[nidx].to_neighbor_id_ == 0) {
      auto const& way = prep_.all_neighbors_[nidx].edge_;
      auto const& dir = prep_.all_neighbors_[nidx].dir_;
      way_in_neighbor_[nidx] = way;
      dir_in_neighbor_[nidx] = dir;
      way_out_neighbor_[nidx] = way;
      dir_out_neighbor_[nidx] = dir;

    } else {
      auto const& to_via_idx = prep_.all_neighbors_[nidx].to_via_id_;
      auto const& to_neighbor_idx = prep_.all_neighbors_[nidx].to_neighbor_id_;
      utl::verify(to_via_idx <= nidx || to_neighbor_idx <= nidx,
                  "Neighbor of id {} has lower neighbors {}, {} as a shortcut",
                  nidx, to_via_idx, to_neighbor_idx);
      way_in_neighbor_[nidx] = way_out_neighbor_[to_via_idx];
      dir_in_neighbor_[nidx] = opposite(dir_out_neighbor_[to_via_idx]); 
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

      // calculate upward costs:
      auto const upper_nc = P::node_cost(params, ways_.r_->node_properties_[next.neighbor_]);
      auto const wc_up = P::way_cost(params, ways_.r_->way_properties_[next.edge_], next.dir_, 
                                     ways_.r_->get_way_node_distance(next.edge_, next.in_way_idx_));

      if (upper_nc == osr::kInfeasible || wc_up == osr::kInfeasible) {
        neighbor_costs_up_[nidx] = osr::kInfeasible;
      } else {
        neighbor_costs_up_[nidx] = wc_up + upper_nc;
      }
      // calculate downward costs:
      auto const lower_nc = P::node_cost(params, ways_.r_->node_properties_[curr.node_]);
      auto const wc_down = P::way_cost(params, ways_.r_->way_properties_[next.edge_], osr::opposite(next.dir_), 
                                       ways_.r_->get_way_node_distance(next.edge_, next.in_way_idx_));

      if (lower_nc == osr::kInfeasible || wc_down == osr::kInfeasible) {
        neighbor_costs_down_[nidx] = osr::kInfeasible;
      } else {
        neighbor_costs_down_[nidx] = wc_down + lower_nc;
      }
    } else { // handle concatenated neighbor (shortcut)
      //  calculate upward costs: 
      auto const& to_via_c_up = neighbor_costs_down_[next.to_via_id_];
      auto const& to_neighbor_c_up = neighbor_costs_up_[next.to_neighbor_id_];

      auto const& from_lower_to_via = way_in_neighbor_[next.to_via_id_];
      auto const& from_via_to_upper = way_in_neighbor_[next.to_neighbor_id_];

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
        neighbor_costs_up_[nidx] = to_via_c_up + via_turn_c_up + to_neighbor_c_up;
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
        neighbor_costs_down_[nidx] = to_via_c_down + turn_angle_c_down + to_neighbor_c_down;    
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
    neighbor_costs_up_.resize(prep_.neighbor_counter_);
    neighbor_costs_down_.resize(prep_.neighbor_counter_);
    way_in_neighbor_.resize(prep_.neighbor_counter_);
    way_out_neighbor_.resize(prep_.neighbor_counter_);
    dir_in_neighbor_.resize(prep_.neighbor_counter_);
    dir_out_neighbor_.resize(prep_.neighbor_counter_);

    for (auto const& node : prep_.neighborhoods_) {
      for (auto const neighbor : node.neighbors_) {
        get_turn_data(neighbor);
        get_neighbor_cost<P>(params, node, neighbor, prep_.all_neighbors_[neighbor]);
      }
    }
    return;
  }

  void run(osr::search_profile const& profile, osr::profile_parameters const& params) {
    return with_valid_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return run<P>(pp);
    });
  }

  osr::ways& ways_;
  mip_proc& prep_;
  osr::vec<osr::cost_t> neighbor_costs_up_;
  osr::vec<osr::cost_t> neighbor_costs_down_;
  osr::vec<osr::way_idx_t> way_in_neighbor_; // store way idx of lower end
  osr::vec<osr::way_idx_t> way_out_neighbor_;// store way idx of higher end
  osr::vec<osr::direction> dir_in_neighbor_;
  osr::vec<osr::direction> dir_out_neighbor_;
};


} // namespace cch