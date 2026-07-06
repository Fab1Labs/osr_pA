#pragma once

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace cch {

template <osr::Profile P, bool EarlyTermination = false>
struct bidir_dijkstra {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  static constexpr auto const kDebug = false;

  struct get_bucket{
    osr::cost_t operator()(label const& l) {return l.cost();}
  };

  void reset(osr::cost_t const max, 
              osr::location const& start_loc,
              osr::location const& end_loc) {
    pq_f_.clear();
    pq_b_.clear();
    pq_f_.n_buckets(max + 1U);
    pq_b_.n_buckets(max + 1U);
    max_reached_f_ = false;
    max_reached_b_ = false;
    cost_f_.clear();
    cost_b_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    mu_ = osr::kInfeasible;
    meet_point_ = meet_point_.invalid();
  }

  void add_start(osr::ways const& w, label const l) {
    if (cost_f_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pq_f_.n_buckets(),
                  "bidir_dijkstra::add_start: label cost exceed max: {} >= {}",
                l.cost(), pq_f_.n_buckets());
      pq_f_.push(l);
    }
  }

  void add_end(osr::ways const& w, label const l) {
    if (cost_b_[l.get_node().get_key()].update(l, l.get_node(), l.cost(), 
                                                node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "END ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pq_b_.n_buckets(),
                  "bidir_dijkstra::add_end: label cost exceed max: {} >= {}",
                l.cost(), pq_b_.n_buckets());
      pq_b_.push(l);
    }
  }

  template <osr::direction PathDir>
  osr::cost_t get_cost(node const n) const {
    if (PathDir == osr::direction::kForward) {
      auto const it = cost_f_.find(n.get_key());
      return it != end(cost_f_) ? it->second.cost(n) : osr::kInfeasible;
    } else {
      auto const it = cost_b_.find(n.get_key());
      return it != end(cost_b_) ? it->second.cost(n) : osr::kInfeasible;
    }
  }

  template <osr::direction SearchDir, bool WithBlocked, osr::direction PathDir>
  bool run_single(P::parameters const& params, 
          osr::ways const& w,
          osr::ways::routing const& r, 
          osr::cost_t const max,
          osr::bitvec<osr::node_idx_t> const* blocked,
          osr::sharing_data const* sharing,
          osr::elevation_storage const* elevations,
          label l,
          osr::dial<label, get_bucket>& pq,
          cost_map& costs) {
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);
    if (get_cost<PathDir>(l.get_node()) < l.cost()) {
      return PathDir == osr::direction::kForward ? !max_reached_f_ : !max_reached_b_;
    }

    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    auto const is_fwd = PathDir == osr::direction::kForward;

    P::template adjacent<SearchDir, WithBlocked>(
      params, r, curr, blocked, sharing, elevations,
      [&](node const neighbor, std::uint32_t const cost, osr::distance_t,
          osr::way_idx_t const way, std::uint16_t, std::uint16_t,
          osr::elevation_storage::elevation, bool const track) {

        if constexpr (kDebug) {
          std::cout << "NEIGHBOR ";
          neighbor.print(std::cout, w);
        }
        
        auto const total = static_cast<std::uint64_t>(curr_cost) + cost;
        if (total >= max && is_fwd) {
          max_reached_f_ = true;
          return;
        }
        if (total >= max && !is_fwd) {
          max_reached_b_ = true;
          return;
        }
        
        if (costs[neighbor.get_key()].update(
                l, neighbor, static_cast<osr::cost_t>(total), curr)) {
          auto next = label{neighbor, static_cast<osr::cost_t>(total)};
          next.track(l, r, way, neighbor.get_node(), track);
          pq.push(std::move(next));

          if constexpr (kDebug) {
            is_fwd ? std::cout << " -> PUSH (fw)\n" : std::cout << " -> PUSH (bw)\n";
          }
        } else {
          if constexpr (kDebug) {
            is_fwd ? std::cout << " -> DOMINATED (fw)\n" : std::cout << " -> DOMINATED (bw)\n";
          }
        }
    
        // check for a potential meetpoint here:
       auto contrary_cost = get_cost<opposite(PathDir)>(neighbor);
        if ((contrary_cost != osr::kInfeasible) && total + contrary_cost < mu_) {
          mu_ = total + contrary_cost;
          if (PathDir == osr::direction::kForward) {
            meet_point_ = neighbor;
          }
        }
      });

    // auto const evaluate_meetpoint = [&](osr::cost_t cost_f, osr::cost_t cost_b,
    //                                     node meetpoint) {
    //   if constexpr (kDebug) {
    //     std::cout << " potential MEETPOINT found by start ";
    //     meetpoint.print(std::cout, w);
    //   }
    //   auto const tentative = static_cast<std::uint64_t>(cost_f) +
    //                          static_cast<std::uint64_t>(cost_b);

    //   if (tentative < mu_) {
    //     meet_point_ = meetpoint;
    //     mu_ = osr::clamp_cost(tentative);
    //   }
    // };

    // auto const handle_meet_point = [&] () {
    //   auto const opposite_cost_map = is_fwd ? &cost_b_ : &cost_f_;
    //   auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
    //   if (opposite_candidate == end(*opposite_cost_map)) {
    //     return;
    //   }
    //   auto const opposite_cost = opposite_candidate->second.cost(curr);
    //   if (opposite_cost != osr::kInfeasible) {
    //     evaluate_meetpoint(curr_cost, opposite_cost, curr);
    //   } else {
    //     auto const pred_it = costs.find(curr.get_key());
    //     if (pred_it == end(costs)) {
    //       return;
    //     }
    //     auto const pred = pred_it->second.pred(curr);
    //     if (!pred.has_value()) {
    //       return;
    //     }
    //     P::template adjacent<opposite(SearchDir), WithBlocked>(
    //         params, r, curr, blocked, sharing, elevations,
    //         [&](node const neighbor, std::uint32_t const, osr::distance_t,
    //             osr::way_idx_t const, std::uint16_t, std::uint16_t,
    //             osr::elevation_storage::elevation const, bool const) {
    //           if (neighbor.get_key() != pred->get_key()) {
    //             return;
    //           }
    //           auto const opposite_it =
    //               opposite_cost_map->find(neighbor.get_key());
    //           if (opposite_it == end(*opposite_cost_map)) {
    //             return;
    //           }
    //           auto const opposite_curr = opposite_it->second.pred(neighbor);
    //           if (!opposite_curr.has_value() ||
    //               opposite_curr->get_key() != curr.get_key()) {
    //             return;
    //           }
    //           auto const opposite_curr_cost =
    //               opposite_candidate->second.cost(*opposite_curr);
    //           auto const pred_cost = get_cost<PathDir>(*pred);
    //           auto const opposite_pred_cost =
    //               opposite_it->second.cost(neighbor);
    //           auto const evaluate_meetpoint_with_potential_u_turn_cost =
    //               [&](osr::cost_t const cost_f, osr::cost_t const cost_b,
    //               )
    //     });
    //   }
    // };

    // handle_end_of_way_meetpoint();
 
    return SearchDir == osr::direction::kForward ? !max_reached_f_ : !max_reached_b_;
  }

  template <osr::direction SearchDir, bool WithBlocked>
  bool run(P::parameters const& params,
           osr::ways const& w,
           osr::ways::routing const& r,
           osr::cost_t const max,
           osr::bitvec<osr::node_idx_t> const* blocked,
           osr::sharing_data const* sharing,
           osr::elevation_storage const* elevations) {
    while (!pq_f_.empty() && !pq_b_.empty()) {

      auto forward_n = pq_f_.pop();
      auto backward_n = pq_b_.pop();

      if (!run_single<SearchDir, WithBlocked, osr::direction::kForward>(
          params, w, r, max, blocked, sharing, elevations, forward_n, pq_f_, cost_f_)) {
        break;
      }

      if (!run_single<opposite(SearchDir), WithBlocked, osr::direction::kBackward>(
          params, w, r, max, blocked, sharing, elevations, backward_n, pq_b_, cost_b_)) {
        break;
      }

      if (get_cost<osr::direction::kForward>(forward_n.get_node()) + 
          get_cost<osr::direction::kBackward>(backward_n.get_node()) >= 
          mu_) { 
        return false;
      }
    }

    return !max_reached_f_ || !max_reached_b_;
  }

  bool run(P::parameters const& params, 
           osr::ways const& w,
           osr::ways::routing const& r, 
           osr::cost_t const max, 
           osr::bitvec<osr::node_idx_t> const* blocked,
           osr::sharing_data const* sharing, 
           osr::elevation_storage const* elevations, 
           osr::direction const dir) {
    if (blocked == nullptr) {
      return dir == osr::direction::kForward
                  ? run<osr::direction::kForward, false>(params, w, r, max, blocked, sharing, elevations)
                  : run<osr::direction::kBackward, false>(params, w, r, max, blocked, sharing, elevations);
    } else {
      return dir == osr::direction::kForward
                  ? run<osr::direction::kForward, true>(params, w, r, max, blocked, sharing, elevations)
                  : run<osr::direction::kBackward, true>(params, w, r, max, blocked, sharing, elevations);
    }
  }

  osr::location start_loc_;
  osr::location end_loc_;
  osr::dial<label, get_bucket> pq_f_{get_bucket{}};
  osr::dial<label, get_bucket> pq_b_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_f_;
  ankerl::unordered_dense::map<key, entry, hash> cost_b_;
  osr::cost_t mu_;
  node meet_point_;
  bool max_reached_f_{};
  bool max_reached_b_{};
};
} // namespace cch