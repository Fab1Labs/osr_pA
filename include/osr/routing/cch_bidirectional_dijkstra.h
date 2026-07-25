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
  static constexpr auto const kGplus = false; // <- Define to run the bidir dijkstra on normal graph or with shortcuts

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

  osr::cost_t get_turn_cost(P::parameters const& params, osr::ways::routing const& r, node const& n, 
                            osr::shortcut_idx_t const& sc) {
    auto const& sc_info = r.in_shortcut_[sc];
    auto const to_way_pos = r.get_way_pos(n.n_, sc_info.way_);
    auto const turn_angle = r.get_turn_angle(n.n_, n.way_, n.dir_, to_way_pos, sc_info.dir_);
    return P::turn_cost(params, turn_angle);
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
          cost_map& costs,
          osr::vec<osr::cost_t> const& sc_costs) {
    auto const is_fwd = PathDir == osr::direction::kForward;
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);
    if (get_cost<PathDir>(l.get_node()) < l.cost()) {
      return PathDir == osr::direction::kForward ? !max_reached_f_ : !max_reached_b_;
    }

    if constexpr (kDebug) {
      is_fwd ? std::cout << "EXTRACT (fw) " : std::cout << "EXTRACT (bw) ";
      l.get_node().print(std::cout, w);
      std::cout << " Importance: " << r.node_importance_[l.get_node().n_];
      std::cout << "\n";
    }

    if(kGplus) {
      auto const& shortcuts = r.node_shortcuts_up_[curr.n_];

      // add all shortcuts to the queue:
      for (auto const& sc : shortcuts) {
        if (sc_costs[sc] == osr::kInfeasible) {
          continue;
        }

        auto const& sc_struct = r.shortcut_properties_[sc];

        if (r.node_importance_[curr.n_] > r.node_importance_[sc_struct.upper_end_]) {
          continue;
        }
        auto const turn_cost = get_turn_cost(params, r, curr, sc);
        auto const total_cost = curr_cost + turn_cost + sc_costs[sc];

        if (total_cost >= max && is_fwd) {
          max_reached_f_ = true;
          break;
        }
        if (total_cost >= max && !is_fwd) {
          max_reached_b_ = true;
          break;
        }

        // create neighbor node from shortcut target
        auto const neighbor = typename P::node{
          sc_struct.upper_end_, 
          r.get_way_pos(sc_struct.upper_end_, r.out_shortcut_[sc].way_), 
          r.out_shortcut_[sc].dir_};

        if constexpr (kDebug) {
          std::cout << "NEIGHBOR ";
          neighbor.print(std::cout, w);
          std::cout << " Importance: " << r.node_importance_[neighbor.n_];
          std::cout << " (SHORTCUT) " << sc << ", COST: " << total_cost;
        }
      
        // push the node to the pq
        if (costs[neighbor.get_key()].update(
            l, neighbor, total_cost, curr)) {
          auto next = label{neighbor, static_cast<osr::cost_t>(total_cost)};
          //next.track(...);
          pq.push(std::move(next));

          if constexpr (kDebug) {
            is_fwd ? std::cout << " -> PUSH (fw)\n" : std::cout << " -> PUSH (bw)\n";
          }
        } else {
          if constexpr (kDebug) {
            is_fwd ? std::cout << " -> DOMINATED (fw)\n" : std::cout << " -> DOMINATED (bw)\n";
          }
        }

        // check for breaking condition:
        auto contrary_cost = get_cost<opposite(PathDir)>(neighbor);
        if ((contrary_cost != osr::kInfeasible) && total_cost + contrary_cost < mu_) {
          mu_ = total_cost + contrary_cost;
          if constexpr (kDebug) { 
            std::cout << "=> MEETING POINT: " << neighbor.n_ << " TOTAL COST: " << mu_ <<"\n";
          }
          meet_point_ = neighbor;
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
        auto contrary_cost = get_cost<opposite(PathDir)>(neighbor);
          if ((contrary_cost != osr::kInfeasible) && total + contrary_cost < mu_) {
            mu_ = total + contrary_cost;
            meet_point_ = neighbor;
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
    while (!pq_f_.empty() && !pq_b_.empty()) {

      auto forward_n = pq_f_.pop();
      auto backward_n = pq_b_.pop();

      if (!run_single<SearchDir, WithBlocked, osr::direction::kForward>(
          params, w, r, max, blocked, sharing, elevations, forward_n, pq_f_, cost_f_, r.shortcut_costs_up_)) {
        break;
      }

      if (!run_single<opposite(SearchDir), WithBlocked, osr::direction::kBackward>(
          params, w, r, max, blocked, sharing, elevations, backward_n, pq_b_, cost_b_, r.shortcut_costs_down_)) {
        break;
      }

      if (get_cost<osr::direction::kForward>(forward_n.get_node()) + 
          get_cost<osr::direction::kBackward>(backward_n.get_node()) >= 
          mu_) { 

        if constexpr (kDebug) {
          std::cout << "TERMINATED: cost(fn) + cost(bn) >= mu\n";
        }
        return false;
      }
    }
    if constexpr (kDebug) {
      std::cout << "TERMINATED: empty priority queues";
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

// ./build/osr-backend -d ./test/aachen -s web  