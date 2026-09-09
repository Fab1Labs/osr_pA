#pragma once

#include "utl/verify.h"
#include "utl/enumerate.h"

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
  static constexpr auto const kGplus = true; // <- Define to run the bidir dijkstra on normal graph or with shortcuts

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
    curr_fw_cost_ = osr::cost_t{0U};
    curr_bw_cost_ = osr::cost_t{0U};
    meet_point_f_ = meet_point_f_.invalid();
    meet_point_b_ = meet_point_b_.invalid();
  }

  void add_start(osr::ways const& w, label const l) {
    if (cost_f_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << " Importance: " << w.r_->node_importance_[l.get_node().n_];
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
        std::cout << " Importance: " << w.r_->node_importance_[l.get_node().n_];
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

  template <osr::direction PathDir>
  bool check_restrictions(osr::ways::routing const& r, 
                          node const& curr, 
                          std::uint16_t const& next_way_pos) {
    return r.is_restricted<PathDir, false>(
            curr.n_, curr.way_, next_way_pos);
  }

  template <osr::direction PathDir>
  std::tuple<osr::cost_t, node> find_opposite(P::parameters const& params, 
                                              node const n,
                                              osr::ways::routing const& r) {
    auto const ways = r.node_ways_[n.n_];
    auto min_cost = osr::kInfeasible;
    auto contr_node = P::node::invalid();
    for (auto w : ways) {
      auto const way_pos = r.get_way_pos(n.n_, w);
      if (check_restrictions<PathDir>(r, n, way_pos)) {
        continue;
      }
      auto op_node = node{n.n_, way_pos, osr::direction::kForward};
      auto op_cost = get_cost<osr::opposite(PathDir)>(op_node);
      if (op_cost != osr::kInfeasible && 
          way_pos == n.way_ && 
          n.dir_ == osr::direction::kBackward) {
        op_cost += params.uturn_penalty_;
      }
      if (op_cost < min_cost) {
        min_cost = op_cost;
        contr_node = op_node;
      }

      op_node = node{n.n_, way_pos, osr::direction::kBackward};
      op_cost = get_cost<osr::opposite(PathDir)>(op_node);
      if (op_cost != osr::kInfeasible && 
          way_pos == n.way_ && 
          n.dir_ == osr::direction::kForward) {
        op_cost += params.uturn_penalty_;
      }
      if (op_cost < min_cost) {
        min_cost = op_cost;
        contr_node = op_node;
      }
    }
    return std::make_tuple(min_cost, contr_node);
  }

  template <osr::direction SearchDir, bool WithBlocked, osr::direction PathDir>
  bool run_single(P::parameters const& params, 
          osr::ways const& w,
          osr::ways::routing const& r, 
          osr::cost_t const max,
          osr::bitvec<osr::node_idx_t> const* blocked,
          osr::sharing_data const* sharing,
          osr::elevation_storage const* elevations,
          osr::dial<label, get_bucket>& pq,
          cost_map& costs) {
    auto const is_fwd = PathDir == osr::direction::kForward;
    auto const l = pq.pop();

    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);

    if (is_fwd) {
      curr_fw_cost_ = curr_cost;
    } else {
      curr_bw_cost_ = curr_cost;
    }

    if (get_cost<PathDir>(l.get_node()) < l.cost()) {
      if constexpr (kDebug) {
        is_fwd ? std::cout << "RETURN (fw) " : std::cout << "RETURN (bw) ";
        curr.print(std::cout, w);
        std::cout << " Expected " << get_cost<PathDir>(l.get_node()) << " but got " << l.cost() << "\n";
      }
      return PathDir == osr::direction::kForward ? !max_reached_f_ : !max_reached_b_;
    }

    if constexpr (kDebug) {
      is_fwd ? std::cout << "EXTRACT (fw) " : std::cout << "EXTRACT (bw) ";
      l.get_node().print(std::cout, w);
      std::cout << " Importance: " << r.node_importance_[l.get_node().n_];
      std::cout << " COST: " << get_cost<PathDir>(l.get_node());
      std::cout << " PQ SIZE: " << pq.size();
      std::cout << "\n";
    }

    if(kGplus) {
      auto const& curr_importance = r.node_importance_[curr.n_];
      auto const& targets = r.sc_targets_[curr_importance];
      auto const& sc_costs = is_fwd ? r.cch_cost_up_[curr_importance] 
                                    : r.cch_cost_down_[curr_importance];
      auto const& sc_properties = is_fwd ? r.cch_sc_up_[curr_importance]
                                         : r.cch_sc_down_[curr_importance];

      // add all shortcuts to the queue:
      for (auto [target, cost, property] : utl::zip(targets, sc_costs, sc_properties)) {
        if (cost == osr::kInfeasible) {
          if constexpr (kDebug) {
            std::cout << "  REJECTED: " << target <<  " with infeasible cost\n";
          }
          continue;
        }

        auto const& entry_node = is_fwd ? property.entry_node_ : property.exit_node_;
        auto const& exit_node = is_fwd ? property.exit_node_ : property.entry_node_;

        if (check_restrictions<PathDir>(r, curr, entry_node.way_)) {
          if constexpr (kDebug) {
            std::cout << "  REJECTED: " << target << " with restriction\n";
          }
          // add possible u turn here to enter the shortcut correctly
          continue;
        }
        utl::verify(target == exit_node.n_, 
                    "Got target: {} but exptected: {}",
                    exit_node.n_, target);
        utl::verify(curr.n_ == entry_node.n_,
                    "Got entry: {} but expected: {}",
                    entry_node.n_, curr.n_);

        auto neighbor_cost = osr::clamp_cost(static_cast<std::uint64_t>(cost) + curr_cost);
        if (curr.way_ == entry_node.way_ && 
            curr.dir_ == osr::opposite(entry_node.dir_)) {
          neighbor_cost += params.uturn_penalty_;
        }

        if (neighbor_cost >= max && is_fwd) {
          max_reached_f_ = true;
          break;
        }
        if (neighbor_cost >= max && !is_fwd) {
          max_reached_b_ = true;
          break;
        }

        auto const neighbor = typename P::node{target, exit_node.way_, exit_node.dir_};

        // if constexpr (kDebug) {
        //   auto path_cost = curr_cost;
        //   for (auto [idx, node] : utl::enumerate(property.nodes_)) {
        //     path_cost += property.costs_[idx];
        //     auto const path_node = typename P::node{
        //         node, r.get_way_pos(node, property.ways_[idx]), property.dirs_[idx]
        //     };
        //     if (node == property.nodes_.back()) {
        //       std::cout << "  NEIGHBOR ";
        //     } else {
        //       std::cout << "  -> ";
        //     }
        //     path_node.print(std::cout, w);
        //     std::cout << " IMPORTANCE: " << r.node_importance_[node];
        //     if (node == property.nodes_.back()) {
        //       std::cout << " COST: " << neighbor_cost;
        //     } else {
        //       std::cout << " COST: " << path_cost;
        //     }
        //     std::cout << " WAY: " << property.ways_[idx];

        //     if (node != property.nodes_.back()) {
        //       std::cout << "\n";
        //     }
        //   }
        // }

        if (costs[neighbor.get_key()].update(
            l, neighbor, neighbor_cost, curr)) {
          auto const hashmap_cost = costs.find(neighbor.get_key());
          auto next = label{neighbor, static_cast<osr::cost_t>(neighbor_cost)};
          next.track(l, r, r.node_ways_[target][exit_node.way_], neighbor.get_node(), false);
          utl::verify(hashmap_cost->second.cost(neighbor) == neighbor_cost, 
              "Expected costs {} but got {}", neighbor_cost, hashmap_cost->second.cost(neighbor));
          utl::verify(get_cost<PathDir>(neighbor) == neighbor_cost,
              "Expected costs {} but got {}", neighbor_cost, get_cost<PathDir>(neighbor));
          pq.push(std::move(next));

          if constexpr (kDebug) {
            is_fwd ? std::cout << " -> PUSH (fw)" : std::cout << " -> PUSH (bw)";
            std::cout << " PQ SIZE: " << pq.size() << "\n";
          }
        } else {
          if constexpr (kDebug) {
            is_fwd ? std::cout << " -> DOMINATED (fw)\n" : std::cout << " -> DOMINATED (bw)\n";
          }
        }
      }

      //auto const contrary_cost = get_cost<osr::opposite(PathDir)>(neighbor);
      auto const [contrary_cost, contrary_node] = find_opposite<PathDir>(params, curr, r);
      auto total = get_cost<PathDir>(curr);
      if constexpr (kDebug) {
        std::cout << " CURR_COST: " << total <<  " CONTR_COST: " << contrary_cost << "\n";
      }
      if ((contrary_cost != osr::kInfeasible) && ((total + contrary_cost) < mu_)) {
        mu_ = total + contrary_cost;
        if constexpr (kDebug) { 
          std::cout << "=> MEETING POINT: " << curr.n_ << " TOTAL COST: " << mu_ <<"\n";
        }
        utl::verify(curr.n_ == contrary_node.n_,
                    "Expected equality of meetpoint nodes for {} and {}",
                    curr.n_, contrary_node.n_);
        if (is_fwd) {
          meet_point_f_ = curr;
          meet_point_b_ = contrary_node;
        } else {
          meet_point_f_ = contrary_node;
          meet_point_b_ = curr;
        }
      }
    } else {
      P::template adjacent<SearchDir, WithBlocked>( // lasse die adjacent drin, wegen optionaler feature flag
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
          auto contrary_cost = get_cost<osr::opposite(PathDir)>(neighbor);
            if ((contrary_cost != osr::kInfeasible) && total + contrary_cost < mu_) {
              mu_ = total + contrary_cost;
              meet_point_f_ = neighbor;
              if constexpr (kDebug) { 
                std::cout << "=> MEETING POINT: " << neighbor.n_ << " TOTAL COST: " << mu_ <<"\n";
              }
            }
      });
    }
 
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
    while (!pq_f_.empty() || !pq_b_.empty()) {

      if (!pq_f_.empty() &&
          !run_single<SearchDir, WithBlocked, osr::direction::kForward>(
              params, w, r, max, blocked, sharing, elevations, pq_f_, cost_f_)) {
        break;
      }

      if (!pq_b_.empty() && 
          !run_single<SearchDir, WithBlocked, osr::direction::kBackward>(
              params, w, r, max, blocked, sharing, elevations, pq_b_, cost_b_)) {
        break;
      }
    
      if (curr_fw_cost_ + curr_bw_cost_ >= mu_) {
        if constexpr (kDebug) {
          std::cout << "TERMINATED: cost(fn) + cost(bn) >= mu\n";
        }
        return false;
      }
    }
    if constexpr (kDebug) {
      std::cout << "TERMINATED: empty priority queues. ";
      std::cout << "Size Forward Queue: " << pq_f_.size() << " ";
      std::cout << "Size Backward Queue: " << pq_b_.size() << "\n";

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
  osr::cost_t curr_fw_cost_;
  osr::cost_t curr_bw_cost_;
  node meet_point_f_;
  node meet_point_b_;
  bool max_reached_f_{};
  bool max_reached_b_{};
};
} // namespace cch

// ./build/osr-backend -d ./test/aachen -s web  